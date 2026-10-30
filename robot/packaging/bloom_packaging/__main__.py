import argparse
import glob
import json
import os
import sys

from . import build, metapackage, version


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='bloom_packaging')
    sub = ap.add_subparsers(dest='cmd', required=True)

    v = sub.add_parser('version', help='SemVer (nbgv) -> Debian version')
    v.add_argument('semver')
    v.add_argument('--revision', default='1')

    b = sub.add_parser('build', help='build every ROS package in a tree into stamped .debs (prints the plan unless --execute)')
    b.add_argument('--src', required=True)
    b.add_argument('--out', required=True)
    b.add_argument('--semver', required=True, help='nbgv get-version -v SemVer2')
    b.add_argument('--revision', default='1')
    b.add_argument('--ros-distro', default='jazzy')
    b.add_argument('--os', default='ubuntu')
    b.add_argument('--codename', default='noble')
    b.add_argument('--exclude', default='', help='comma-separated ROS package names to skip')
    b.add_argument('--execute', action='store_true', help='really build (needs root, debhelper, bloom: use a CI container)')
    b.add_argument('--install-built', action='store_true', help='apt-get install each .deb after building (CI only)')

    m = sub.add_parser('meta', help='build the metapackage pinning every .deb in a directory')
    m.add_argument('--name', required=True)
    m.add_argument('--semver', required=True)
    m.add_argument('--revision', default='1')
    m.add_argument('--from-dir', required=True, help='directory of .debs to pin at their exact versions')
    m.add_argument('--out', required=True)
    m.add_argument('--description', default='Pins the exact versions of the Bloom robot software.')
    m.add_argument('--extra-depends', default='', help='comma-separated, verbatim (e.g. libportaudio2)')
    m.add_argument('--env-file', action='append', default=[], help='created empty, root-only, if missing (never overwritten)')

    args = ap.parse_args(argv)

    if args.cmd == 'version':
        print(version.semver_to_debian(args.semver, args.revision))
        return 0

    deb_version = version.semver_to_debian(args.semver, args.revision)
    if args.cmd == 'build':
        exclude = {x for x in args.exclude.split(',') if x}
        if not args.execute:
            print(json.dumps(build.plan(args.src, args.ros_distro, deb_version, exclude), indent=2))
            print(f'(plan only; version {deb_version}. Re-run with --execute in a CI container to build.)', file=sys.stderr)
            return 0
        for deb in build.execute(args.src, args.out, args.ros_distro, args.os, args.codename, deb_version,
                                 exclude, args.install_built):
            print(deb)
        return 0

    pins = {}
    for deb in sorted(glob.glob(os.path.join(args.from_dir, '*.deb'))):
        pkg, ver = metapackage.read_deb_field(deb, 'Package'), metapackage.read_deb_field(deb, 'Version')
        if pkg and ver and pkg != args.name:
            pins[pkg] = ver
    spec = metapackage.MetaSpec(args.name, deb_version, args.description, pins,
                                [x.strip() for x in args.extra_depends.split(',') if x.strip()], args.env_file)
    print(metapackage.build(spec, args.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
