/**
 * 语音传输 — FFE1 与摇杆共用写入特征
 * 包格式: [0x0B, seq_hi, seq_lo, pcm...]  约 180B PCM, 8kHz s16le mono
 */
const WRITE_UUID = '0000FFE1-0000-1000-8000-00805F9B34FB'
const SAMPLE_RATE = 8000
const CHUNK_PCM = 180

function buildMpAudioPacket(seq, pcmU8) {
  const pcm = pcmU8.length > CHUNK_PCM ? pcmU8.subarray(0, CHUNK_PCM) : pcmU8
  const out = new Uint8Array(3 + pcm.length)
  out[0] = 0x0b
  out[1] = (seq >> 8) & 0xff
  out[2] = seq & 0xff
  out.set(pcm, 3)
  return out.buffer
}

async function sendSoundOn(deviceId, serviceId) {
  await writeText(deviceId, serviceId, WRITE_UUID, 'sound ON')
}

async function sendSoundOff(deviceId, serviceId) {
  await writeText(deviceId, serviceId, WRITE_UUID, 'sound OFF')
}

async function sendAudioChunk(deviceId, serviceId, seq, pcmBuffer) {
  const frame = buildMpAudioPacket(seq, new Uint8Array(pcmBuffer))
  return new Promise((ok, fail) => {
    wx.writeBLECharacteristicValue({
      deviceId,
      serviceId,
      characteristicId: WRITE_UUID,
      value: frame,
      writeType: 'writeNoResponse',
      success: ok,
      fail,
    })
  })
}
