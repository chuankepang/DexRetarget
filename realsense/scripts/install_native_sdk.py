"""Unpack official Ubuntu 26.04 SDK packages under /data/Software/realsense.

This is a private SDK installation, not a dpkg system installation. No package
maintainer scripts, kernel patches, firmware updates or udev changes are run.
Package hashes are checked against the official HTTPS package index.
"""

import gzip
import hashlib
import json
import pathlib
import shlex
import subprocess
import urllib.request

ROOT = pathlib.Path("/data/Software/realsense")
REPO = "https://librealsense.realsenseai.com/Debian/apt-repo/"
VERSION = "2.58.4-0~realsense.19928"
PACKAGES = ["librealsense2", "librealsense2-gl", "librealsense2-utils", "librealsense2-dev"]


def main():
    index = gzip.decompress(urllib.request.urlopen(
        REPO + "dists/resolute/main/binary-amd64/Packages.gz", timeout=60
    ).read()).decode()
    entries = []
    for block in index.split("\n\n"):
        entry = dict(line.split(": ", 1) for line in block.splitlines()
                     if ": " in line and not line.startswith(" "))
        if entry.get("Package") in PACKAGES and entry.get("Version") == VERSION:
            entries.append(entry)
    if {e["Package"] for e in entries} != set(PACKAGES):
        raise RuntimeError("The pinned SDK package set is not available")
    downloads, sdk, bindir = ROOT / "downloads", ROOT / "sdk", ROOT / "bin"
    for path in [downloads, sdk, bindir]:
        path.mkdir(parents=True, exist_ok=True)
    manifest = []
    for entry in entries:
        url = REPO + entry["Filename"]
        data = urllib.request.urlopen(url, timeout=120).read()
        if hashlib.sha256(data).hexdigest() != entry["SHA256"]:
            raise RuntimeError(f"SHA256 mismatch: {entry['Package']}")
        archive = downloads / pathlib.Path(entry["Filename"]).name
        archive.write_bytes(data)
        subprocess.run(["dpkg-deb", "-x", str(archive), str(sdk)], check=True)
        manifest.append({k: entry[k] for k in ["Package", "Version", "Filename", "SHA256"]})
        print(f"Installed {entry['Package']} {VERSION}", flush=True)
    # Scope shared-library lookup to each launcher, never global LD_LIBRARY_PATH.
    libs = sdk / "usr/lib/x86_64-linux-gnu"
    for binary in (sdk / "usr/bin").iterdir():
        if binary.name.startswith(("rs-", "realsense-")):
            launcher = bindir / binary.name
            launcher.write_text(
                '#!/bin/sh\nexport LD_LIBRARY_PATH=' + shlex.quote(str(libs))
                + '${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\nexec '
                + shlex.quote(str(binary)) + ' "$@"\n'
            )
            launcher.chmod(0o755)
    (ROOT / "native-sdk-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
