#!/usr/bin/env bash
# Build the LRV end to end: Blender model -> visuals USD -> physics USD -> Newton test -> renders.
# Draft iteration: LRV_DRAFT=1 ./pipeline.sh ... (512 px textures, everything capped)
# Usage: ./pipeline.sh [step ...] with steps from [textures|build|package|urdf|mjcf|sim|render|hero|assets|update|release|example|view|all]   (default: all)
set -euo pipefail
cd "$(dirname "$0")"
export BLENDER=${BLENDER:-blender}   # Blender 5.2 executable (override with BLENDER=/path/to/blender)
PY=.venv/bin/python

textures() { $PY scripts/textures.py; $PY scripts/crew_textures.py; $PY scripts/eq_textures.py; }
build()  { $BLENDER -b --factory-startup -P scripts/build.py -- --out build/lrv.blend; }
sim()    { $PY scripts/sim_newton.py --usd build/asset/lrv/lrv.usda --record build/sim/run.npz; }
render() { $BLENDER -b build/lrv.blend -P scripts/render.py -- --tag latest --views 34,side,front,wheel; }
urdf()   { $BLENDER -b build/lrv.blend -P scripts/export_urdf.py -- --outdir build/urdf; }
mjcf()   { $PY scripts/export_mjcf.py --validate; }
package() { $BLENDER -b build/lrv.blend -P scripts/export_visuals.py -- --outdir build/usd/full --lod full
            $BLENDER -b build/lrv.blend -P scripts/export_visuals.py -- --outdir build/usd/proxy --lod proxy
            $PY scripts/package_usd.py --thumbnail build/thumb_256.png; }
hero()   { rm -rf build/hero build/gallery   # docs/images is only replaced once every shot has rendered
           $BLENDER -b build/lrv.blend -P scripts/hero.py -- --out build/hero
           $PY scripts/gallery.py --renders build/hero --out build/gallery && rsync -a --delete build/gallery/ docs/images/; }
example() { $PY examples/newton/example_robot_lrv.py --speed 6; }
# Copy build outputs into the committed assets/ folder (Git LFS). LRV_ASSETS picks formats (default: all three).
assets() { mkdir -p assets
           for f in ${LRV_ASSETS:-usd urdf mjcf blend}; do
             case $f in
               usd)  rsync -a --delete build/asset/lrv/ assets/lrv/ ;;
               urdf) rsync -a --delete build/urdf/ assets/urdf/ ;;
               mjcf) rsync -a --delete build/mjcf/ assets/mjcf/ ;;
               blend) $BLENDER -b build/lrv.blend -P scripts/export_blend.py -- --out assets/blender/lrv.blend ;;
             esac; echo "assets: updated $f"
           done; }
update() { build; package; urdf; mjcf; assets; }   # rebuild every format and refresh assets/
# Prepare a release for the version in VERSION: rebuild everything, refresh assets/ (stamped with the version)
# and build the same zips CI will publish into build/release/v<VERSION>/ for a local check. Publishing is done by
# .github/workflows/release.yml when the tag is pushed.
release() { update; $PY scripts/make_release.py; v=$(cat VERSION)
            echo "next: git add -A assets VERSION && git commit -m \"Release v$v\" && git tag v$v && git push origin main v$v"; }
view()   { $PY scripts/sim_newton.py --usd build/asset/lrv/lrv.usda --viewer gl; }

for step in "${@:-all}"; do
  case $step in
    all) textures; build; package; urdf; mjcf; assets; sim; render ;;
    *) $step ;;
  esac
done
