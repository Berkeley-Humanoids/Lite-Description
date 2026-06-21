"""Fetch and cache robot description assets from an upstream GitHub repository."""

from pathlib import Path
import shutil
import tempfile
import threading
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen
import zipfile


_FETCH_LOCK = threading.Lock()

# Robots live under robots/<robot>/ (the lite_description ament package keeps
# its per-robot assets in a robots/ subdir, franka_description style).
ROOT_DIR = "robots"


def load(
    path: str | Path,
    repo_url: str = "https://github.com/Berkeley-Humanoids/Lite-Description",
    cache_dir: str | Path = "data/",
    timeout: float = 20.0,
) -> Path:
    """Load a robot description file from GitHub, or return it from the local cache.

    This function does the following under the hood:

    1. If ``cache_dir / path`` already exists, return its resolved path.
    2. Otherwise download the repository ZIP (GitHub API), extract the
       ``robots/<robot>/`` subtree into ``cache_dir``, then return the resolved path.

    Args:
        path (`str` | `pathlib.Path`):
            Relative path inside the repo, e.g. ``"robots/lite/urdf/lite.urdf"``.
            Must start with ``robots/<robot>/`` and must not be absolute.
        repo_url (`str`, *optional*):
            Standard ``https://github.com/owner/repo`` URL; only ``github.com`` is supported.
        cache_dir (`str` | `pathlib.Path`, *optional*):
            Root directory for cached ``robots/...``. Defaults to ``"data/"``.
        timeout (`float`, *optional*):
            HTTP timeout in seconds for the archive download.

    Returns:
        `pathlib.Path`: Absolute path to the cached file.
    """

    cache_dir = Path(cache_dir)
    robot_name, relative_asset_path = parse_upstream_asset_path(path)
    resolved_path = (cache_dir / relative_asset_path).resolve()
    if resolved_path.exists():
        return resolved_path

    with _FETCH_LOCK:
        if not resolved_path.exists():
            fetch_robot_description(robot_name, repo_url, cache_dir, timeout)

    if resolved_path.exists():
        return resolved_path

    raise FileNotFoundError(
        "Robot description asset is still missing after fetching "
        f"'{robot_name}': {resolved_path}"
    )


def parse_upstream_asset_path(path: str | Path) -> tuple[str, Path]:
    asset_path = Path(path).expanduser()
    if asset_path.is_absolute():
        raise ValueError(
            "Robot description asset paths must be relative upstream paths like "
            f"'robots/lite/urdf/lite.urdf', got {asset_path}"
        )

    parts = asset_path.parts
    if len(parts) < 3 or parts[0] != ROOT_DIR:
        raise ValueError(
            "Robot description asset paths must be of the form "
            f"'{ROOT_DIR}/<robot>/...', got {asset_path}"
        )

    return parts[1], asset_path


def fetch_robot_description(
    robot_name: str,
    repo_url: str,
    cache_dir: Path,
    timeout: float,
) -> None:
    cache_dir.mkdir(parents=True, exist_ok=True)
    target_dir = cache_dir / ROOT_DIR / robot_name

    with tempfile.TemporaryDirectory(prefix=f"robot_assets_{robot_name}_") as tmp:
        archive_path = Path(tmp) / "repo.zip"
        extracted_dir = Path(tmp) / robot_name
        download_archive(archive_path, repo_url, timeout)
        extract_robot_directory(archive_path, robot_name, extracted_dir, repo_url)
        shutil.copytree(extracted_dir, target_dir, dirs_exist_ok=True)


def download_archive(destination: Path, repo_url: str, timeout: float) -> None:
    request = Request(
        build_archive_url(repo_url),
        headers={
            "Accept": "application/vnd.github+json",
            "User-Agent": "robot-description-cache",
        },
    )

    try:
        with (
            urlopen(request, timeout=timeout) as response,
            destination.open("wb") as archive_file,
        ):
            shutil.copyfileobj(response, archive_file)
    except HTTPError as exc:
        raise RuntimeError(
            "Failed to download robot descriptions archive from GitHub. "
            f"HTTP {exc.code}: {exc.reason}"
        ) from exc
    except URLError as exc:
        raise RuntimeError(
            "Failed to download robot descriptions archive from GitHub. "
            f"Reason: {exc.reason}"
        ) from exc


def extract_robot_directory(
    archive_path: Path,
    robot_name: str,
    destination: Path,
    repo_url: str,
) -> None:
    found_robot = False

    with zipfile.ZipFile(archive_path) as archive:
        for info in archive.infolist():
            parts = Path(info.filename).parts
            try:
                robots_index = parts.index(ROOT_DIR)
            except ValueError:
                continue

            if len(parts) <= robots_index + 1 or parts[robots_index + 1] != robot_name:
                continue

            found_robot = True
            relative_path = Path(*parts[robots_index + 2 :])
            output_path = destination / relative_path

            if info.is_dir():
                output_path.mkdir(parents=True, exist_ok=True)
                continue

            output_path.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(info) as source_file, output_path.open("wb") as target_file:
                shutil.copyfileobj(source_file, target_file)

    if not found_robot:
        raise FileNotFoundError(f"Robot '{robot_name}' was not found in {repo_url}.")


def build_archive_url(repo_url: str) -> str:
    parsed_url = urlparse(repo_url.removesuffix(".git"))
    path_parts = [part for part in parsed_url.path.split("/") if part]
    if parsed_url.netloc != "github.com" or len(path_parts) != 2:
        raise ValueError(
            "Only standard GitHub repository URLs are supported. "
            f"Could not derive archive URL from {repo_url!r}."
        )

    owner, repo = path_parts
    return f"https://api.github.com/repos/{owner}/{repo}/zipball"
