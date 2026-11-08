#!/bin/bash
# Install one apt package list, with the .deb files kept in an Actions cache.
#   apt-install.sh key LIST                update, then write the cache key
#   apt-install.sh install LIST CACHE_DIR  download under a cap, then install
set -euo pipefail

# Acquire::http::Timeout only fires after a gap with no bytes at all.  A
# mirror that trickles never trips it: azure.archive.ubuntu.com at times
# sends 40-50 kB/s, and the Install step then ran to its 45 minute timeout.
# So each download attempt has its own cap.  The slowest healthy Install
# step took 237 s for update, download and dpkg together; 480 s is twice
# that.  Three attempts and dpkg fit in the 35 minute step timeout.
CAP=480
OPTS=(-o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)

# One attempt, run as root under timeout by "install" below, so a single
# cap covers both the update and the download.  timeout signals its whole
# process group, which reaches apt-get and its http methods.
if [[ ${1:-} == fetch ]]; then
  if [[ $2 == update ]]; then
    apt-get "${OPTS[@]}" update
  fi
  exec apt-get "${OPTS[@]}" install --download-only -y "${@:3}"
fi

list=$2
mapfile -t pkgs < "$list"

case $1 in
key)
  sudo apt-get "${OPTS[@]}" update
  # The hash of the resolved set is only the suffix: restore-keys match the
  # prefix, so after a package update the previous debs are restored and
  # apt downloads only the difference.
  hash=$(apt-get -s install -y "${pkgs[@]}" | { grep '^Inst ' || true; } |
    LC_ALL=C sort | sha256sum | cut -c1-16)
  prefix=$(basename "$list" .list)-${ImageOS:?}-
  {
    echo "key=$prefix${ImageVersion:?}-$hash"
    echo "prefix-image=$prefix${ImageVersion:?}-"
    echo "prefix-os=$prefix"
  } | tee -a "$GITHUB_OUTPUT"
  ;;
install)
  cache=$3
  mkdir -p "$cache"
  sudo find "$cache" -name '*.deb' -exec cp -t /var/cache/apt/archives {} +
  # The download runs apart from dpkg, so a cap never kills dpkg halfway.
  # Attempts 2 and 3 drop the Azure line from the runner's mirror list and
  # update, so apt fetches from the next mirror (archive.ubuntu.com).
  for n in 1 2 3; do
    upd=no
    if ((n > 1)); then
      sudo sed -i '/azure\.archive\.ubuntu\.com/d' /etc/apt/apt-mirrors.txt
      upd=update
    fi
    echo "apt-install: download attempt $n, cap ${CAP}s"
    if sudo timeout -k 20 "$CAP" "$0" fetch "$upd" "${pkgs[@]}"; then
      break
    fi
    echo "apt-install: download attempt $n failed"
    if ((n == 3)); then
      exit 1
    fi
  done
  # Keep only this install's debs, or old versions pile up in every new
  # entry.  With no debs in its archive directory, --print-uris lists them all.
  find "$cache" -name '*.deb' -delete
  apt-get -o Dir::Cache::Archives="$cache" --print-uris install -y "${pkgs[@]}" |
    sed -n "s|^'[^']*' \([^ ]*\) .*|/var/cache/apt/archives/\1|p" |
    xargs -r cp -t "$cache"
  sudo apt-get "${OPTS[@]}" install --no-download -y "${pkgs[@]}"
  ;;
*)
  echo "usage: $0 key LIST | install LIST CACHE_DIR" >&2
  exit 2
  ;;
esac
