#!/bin/bash
# 根据 ~/cyclonedds.xml + 当前网卡 IP 生成运行时配置（无组播时需本机 Peer）
# 可直接执行，也可 source。
_HOME_DIR="${BIRD_HOME:-${HOME:-/home/nvidia}}"
_BASE_XML="${_HOME_DIR}/cyclonedds.xml"
_RUNTIME_DDS="${_HOME_DIR}/.config/bird/cyclonedds.runtime.xml"
_DDS_IFACE="wlan0"
_PEER_EXTRA="192.168.98.189"

if [ -f "${_BASE_XML}" ]; then
  _if="$(grep -oP '(?<=NetworkInterfaceAddress>)[^<]+' "${_BASE_XML}" 2>/dev/null | head -1 || true)"
  _DDS_IFACE="${_if:-wlan0}"
  _peers="$(grep -oP '(?<=Peer address=")[^"]+' "${_BASE_XML}" 2>/dev/null | grep -v '^127\.0\.0\.1$' || true)"
  if [ -n "${_peers}" ]; then
    _PEER_EXTRA="$(printf '%s\n' ${_peers} | tr '\n' ' ')"
  fi
fi

_local_ip="$(ip -4 -o addr show "${_DDS_IFACE}" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1 || true)"

mkdir -p "$(dirname "${_RUNTIME_DDS}")"
{
  echo '<?xml version="1.0" encoding="UTF-8"?>'
  echo '<CycloneDDS><Domain><General>'
  echo "  <NetworkInterfaceAddress>${_DDS_IFACE}</NetworkInterfaceAddress>"
  echo '  <AllowMulticast>false</AllowMulticast>'
  echo '</General><Discovery>'
  echo '  <ParticipantIndex>auto</ParticipantIndex>'
  echo '  <Peers>'
  echo '    <Peer address="127.0.0.1"/>'
  if [ -n "${_local_ip}" ]; then
    echo "    <Peer address=\"${_local_ip}\"/>"
  fi
  for _p in ${_PEER_EXTRA}; do
    if [ -n "${_p}" ] && [ "${_p}" != "${_local_ip}" ] && [ "${_p}" != "127.0.0.1" ]; then
      echo "    <Peer address=\"${_p}\"/>"
    fi
  done
  echo '  </Peers>'
  echo '  <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>'
  echo '</Discovery></Domain></CycloneDDS>'
} > "${_RUNTIME_DDS}"

export CYCLONEDDS_URI="file://${_RUNTIME_DDS}"
export BIRD_DDS_IFACE="${_DDS_IFACE}"
export BIRD_DDS_LOCAL_IP="${_local_ip}"
if [ "${1:-}" = "--export" ]; then
  echo "export CYCLONEDDS_URI='file://${_RUNTIME_DDS}'"
  echo "export BIRD_DDS_IFACE='${_DDS_IFACE}'"
  echo "export BIRD_DDS_LOCAL_IP='${_local_ip}'"
fi
unset _HOME_DIR _BASE_XML _RUNTIME_DDS _DDS_IFACE _PEER_EXTRA _if _peers _p
# 注意: 勿 unset _local_ip 若调用方需要；此处已写入 XML 与 BIRD_DDS_LOCAL_IP
unset _local_ip
