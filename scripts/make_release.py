"""Build the GitHub release files for the current VERSION from assets/ (run after ./pipeline.sh update).

Run: .venv/bin/python scripts/make_release.py [--out build/release]

Writes build/release/v<VERSION>/:
  lrv-usd.zip     lrv/ (USD asset: lrv.usda, payloads/, textures)  + LICENSE, NOTICE
  lrv-urdf.zip    urdf/ (lrv.urdf, meshes/)                         + LICENSE, NOTICE
  lrv-mjcf.zip    mjcf/ (lrv.xml, lrv_scene.xml, meshes/, textures/) + LICENSE, NOTICE
  lrv-blend.zip   blender/lrv.blend (textures packed)               + LICENSE, NOTICE
  SHA256SUMS      checksums of the zips
  RELEASE_NOTES.md  draft notes for `gh release create`
File names carry no version, so https://github.com/<owner>/<repo>/releases/latest/download/lrv-usd.zip always
points at the newest release.
"""
import argparse
import hashlib
import os
import zipfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FORMATS = {"usd": "lrv", "urdf": "urdf", "mjcf": "mjcf", "blend": "blender"}   # zip name -> assets/ folder


def zip_dir(src_dir, arc_root, zip_path, license_path, notice_path):
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for dirpath, dirnames, files in os.walk(src_dir):
            dirnames.sort()
            for f in sorted(x for x in files if not x.endswith(".blend1")):
                p = os.path.join(dirpath, f)
                z.write(p, os.path.join(arc_root, os.path.relpath(p, src_dir)))
        z.write(license_path, "LICENSE")
        z.write(notice_path, "NOTICE")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "release"))
    a = ap.parse_args()
    version = open(os.path.join(ROOT, "VERSION")).read().strip()
    out = os.path.join(a.out, f"v{version}")
    os.makedirs(out, exist_ok=True)
    lic = os.path.join(ROOT, "assets", "LICENSE")
    notice = os.path.join(ROOT, "assets", "NOTICE")
    sums = []
    for name, folder in FORMATS.items():
        src = os.path.join(ROOT, "assets", folder)
        if not os.path.isdir(src):
            raise SystemExit(f"missing assets/{folder}: run ./pipeline.sh update first")
        zp = os.path.join(out, f"lrv-{name}.zip")
        zip_dir(src, folder, zp, lic, notice)
        sums.append(f"{sha256(zp)}  {os.path.basename(zp)}")
        print(f"wrote {zp} ({os.path.getsize(zp) / 1e6:.0f} MB)")
    with open(os.path.join(out, "SHA256SUMS"), "w") as f:
        f.write("\n".join(sums) + "\n")
    with open(os.path.join(out, "RELEASE_NOTES.md"), "w") as f:
        f.write(f"Apollo LRV asset v{version}.\n\n"
                "| File | Contents |\n|---|---|\n"
                "| `lrv-usd.zip` | SimReady/Isaac-style USD asset (`lrv/lrv.usda`) with all variants and textures |\n"
                "| `lrv-urdf.zip` | URDF (`urdf/lrv.urdf`) with OBJ meshes and textures |\n"
                "| `lrv-mjcf.zip` | MJCF (`mjcf/lrv.xml`, `mjcf/lrv_scene.xml`) with meshes and textures |\n"
                "| `lrv-blend.zip` | Blender source model (`blender/lrv.blend`), textures packed |\n\n"
                "Assets are licensed CC BY 4.0 (LICENSE and NOTICE in each zip). Checksums: `SHA256SUMS`.\n")
    print(f"release files in {out}")


if __name__ == "__main__":
    main()
