"""Fetch the documented CAD references; verify formats and record SHA-256 hashes."""

import concurrent.futures
import hashlib
import json
import subprocess
import zipfile
from datetime import UTC, datetime
from pathlib import Path

ASSETS = Path(__file__).parent / "assets/electronics"


def fetch(asset):
    target = ASSETS / asset["path"]
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".part")
    urls = [asset["url"]]
    if asset["component"] == "regulator_d24v50f5":
        urls += [
            asset["url"].replace("www.pololu.com", host)
            for host in ["pololu.com", "a.pololu-files.com"]
        ]
    error = ""
    for url in urls:
        result = subprocess.run(
            [
                "curl",
                "-L",
                "--fail",
                "--silent",
                "--show-error",
                "--connect-timeout",
                "10",
                "--max-time",
                "60",
                "-o",
                str(temporary),
                url,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode:
            error = result.stderr.strip()
            continue
        try:
            data = temporary.read_bytes()
            suffix = target.suffix.lower()
            if suffix == ".step" and not (
                data.lstrip().startswith(b"ISO-10303-21;")
                and b"END-ISO-10303-21;" in data[-200:]
            ):
                raise ValueError("Response is not a complete STEP model")
            if suffix == ".fcstd":
                with zipfile.ZipFile(temporary) as archive:
                    if "Document.xml" not in archive.namelist() or archive.testzip():
                        raise ValueError("Invalid FreeCAD archive")
            digest = hashlib.sha256(data).hexdigest()
            if asset.get("sha256") and digest != asset["sha256"]:
                raise ValueError("Source hash mismatch")
            temporary.replace(target)
            asset.update(
                status="downloaded", sha256=digest, bytes=len(data), fetched_url=url
            )
            asset.pop("error", None)
            print(asset["path"], len(data), flush=True)
            return asset
        except (ValueError, zipfile.BadZipFile) as exc:
            error = str(exc)
    temporary.unlink(missing_ok=True)
    # Keep a previously verified file if the remote service is unavailable.
    if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() == asset.get(
        "sha256"
    ):
        asset["refresh_error"] = error
        return asset
    asset.update(status="unavailable", error=error)
    print("UNAVAILABLE", asset["component"], error, flush=True)
    return asset


def main():
    path = ASSETS / "sources.json"
    manifest = json.loads(path.read_text())
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        manifest["assets"] = list(pool.map(fetch, manifest["assets"]))
    manifest["checked_utc"] = datetime.now(UTC).isoformat()
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    return int(any(a["status"] != "downloaded" for a in manifest["assets"]))


if __name__ == "__main__":
    raise SystemExit(main())
