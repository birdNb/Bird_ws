/**
 * Bird BLE 指令参考 — 与 BLE_PROTOCOL.md 完全一致
 * FFE0/FFE1/FFE2 | 摇杆 20Hz | 可选 ,N:序号 保活
 */
const SERVICE_UUID = '0000FFE0-0000-1000-8000-00805F9B34FB'
const WRITE_UUID = '0000FFE1-0000-1000-8000-00805F9B34FB'
const NOTIFY_UUID = '0000FFE2-0000-1000-8000-00805F9B34FB'
const TARGET_NAME = 'Bird_BLE_Test'
const STICK_INTERVAL_MS = 50 // 20Hz
const STICK_DEADZONE = 10 // UI -100~100 刻度
const STICK_Z_SCALE = 1.5 // 右转 Z 满量程 ±1.5
const CMD_COOLDOWN_MS = 800
const CHEER_COOLDOWN_MS = 8000

function normUuid(u) {
  return (u || '').replace(/-/g, '').toLowerCase()
}

function uuidHit(u, needle) {
  const n = normUuid(u)
  return n === needle || n.includes(needle)
}

function ab2str(buffer) {
  const arr = new Uint8Array(buffer)
  let s = ''
  for (let i = 0; i < arr.length; i++) s += String.fromCharCode(arr[i])
  try { return decodeURIComponent(escape(s)) } catch (e) { return s }
}

function str2ab(text) {
  const buf = new ArrayBuffer(text.length)
  const v = new Uint8Array(buf)
  for (let i = 0; i < text.length; i++) v[i] = text.charCodeAt(i)
  return buf
}

/** 死区：内部 -100~100，|n|<10 → 0 */
function applyDeadzone(axis100) {
  const n = Math.round(axis100)
  if (Math.abs(n) < STICK_DEADZONE) return 0
  return Math.max(-100, Math.min(100, n)) / 100
}

/** lx=UI横(右+), ly=UI纵(上为负) → 协议 X前后/Y左右/Z右转 */
function formatStick(lx, ly, rz) {
  const x = applyDeadzone(-ly * 100)
  const y = applyDeadzone(lx * 100)
  let z = applyDeadzone(rz * 100) * STICK_Z_SCALE
  z = Math.max(-STICK_Z_SCALE, Math.min(STICK_Z_SCALE, z))
  return `X:${x.toFixed(2)},Y:${y.toFixed(2)},Z:${z.toFixed(2)}`
}

function delay(ms) {
  return new Promise((r) => setTimeout(r, ms))
}

function isIOS() {
  try { return wx.getSystemInfoSync().platform === 'ios' } catch (e) { return false }
}

Page({
  data: { deviceId: '', lx: 0, ly: 0, rz: 0, gattReady: false, robotIp: '', battery: -1, fsmState: -1 },

  _serviceId: '',
  _writeCharId: '',
  _notifyCharId: '',
  _writeNoResponse: false,
  _sendTimer: null,
  _stickSeq: 0,
  _lastCmdAt: 0,
  _lastCheerAt: 0,

  onUnload() {
    this.stopSendLoop()
    if (this.data.deviceId) wx.closeBLEConnection({ deviceId: this.data.deviceId })
  },

  openAdapter() {
    wx.openBluetoothAdapter({
      mode: 'central',
      success: () => this.startScan(),
    })
  },

  startScan() {
    wx.startBluetoothDevicesDiscovery({
      services: [SERVICE_UUID],
      allowDuplicatesKey: true,
      success: () => {
        wx.onBluetoothDeviceFound((res) => {
          res.devices.forEach((d) => {
            if (this.data.deviceId) return
            const name = (d.name || d.localName || '').toLowerCase()
            if (!name.includes('bird_ble') && !uuidHit((d.advertisServiceUUIDs || [])[0], 'ffe0')) return
            this.setData({ deviceId: d.deviceId })
            wx.stopBluetoothDevicesDiscovery({})
            this.establishBleLink(d.deviceId)
          })
        })
      },
    })
  },

  async establishBleLink(deviceId) {
    await this._createConnection(deviceId)
    await delay(isIOS() ? 2000 : 800)
    if (wx.setBLEMTU) wx.setBLEMTU({ deviceId, mtu: 247 })
    await this._discoverGatt(deviceId)
    await this._subscribeNotify(deviceId)
    await this._write('M_default', true)
    await delay(300)
    this.setData({ gattReady: true })
    this.startSendLoop()
  },

  _createConnection(deviceId) {
    return new Promise((ok, fail) => {
      wx.createBLEConnection({ deviceId, timeout: 10000, success: () => ok(), fail: fail })
    })
  },

  _discoverGatt(deviceId) {
    return new Promise((ok, fail) => {
      wx.getBLEDeviceServices({
        deviceId,
        success: (r) => {
          const s = r.services.find((x) => uuidHit(x.uuid, 'ffe0'))
          if (!s) return fail(new Error('no FFE0'))
          this._serviceId = s.uuid
          wx.getBLEDeviceCharacteristics({
            deviceId,
            serviceId: this._serviceId,
            success: (c) => {
              const w = c.characteristics.find((x) => uuidHit(x.uuid, 'ffe1'))
              const n = c.characteristics.find((x) => uuidHit(x.uuid, 'ffe2'))
              if (!w) return fail(new Error('no FFE1'))
              this._writeCharId = w.uuid
              this._notifyCharId = n ? n.uuid : ''
              const p = w.properties || {}
              this._writeNoResponse = !!(p.writeNoResponse || p.writeWithoutResponse)
              ok()
            },
            fail,
          })
        },
        fail,
      })
    })
  },

  _subscribeNotify(deviceId) {
    if (!this._notifyCharId) return Promise.resolve()
    wx.onBLECharacteristicValueChange((ev) => this.onNotify(ab2str(ev.value)))
    return new Promise((ok, fail) => {
      wx.notifyBLECharacteristicValueChange({
        deviceId,
        serviceId: this._serviceId,
        characteristicId: this._notifyCharId,
        state: true,
        success: ok,
        fail,
      })
    })
  },

  _write(text, requireResponse) {
    const deviceId = this.data.deviceId
    return new Promise((ok, fail) => {
      const req = {
        deviceId,
        serviceId: this._serviceId,
        characteristicId: this._writeCharId,
        value: str2ab(text),
        success: ok,
        fail,
      }
      if (!requireResponse && this._writeNoResponse) req.writeType = 'writeNoResponse'
      wx.writeBLECharacteristicValue(req)
    })
  },

  sendStick() {
    const { lx, ly, rz } = this.data
    this._stickSeq += 1
    const base = formatStick(lx, ly, rz)
    const text = `${base},N:${this._stickSeq}`
    this._write(text, false).catch(() => {})
  },

  startSendLoop() {
    this.stopSendLoop()
    this._sendTimer = setInterval(() => this.sendStick(), STICK_INTERVAL_MS)
  },

  stopSendLoop() {
    if (this._sendTimer) clearInterval(this._sendTimer)
    this._sendTimer = null
  },

  onNotify(text) {
    if (text.startsWith('ACK:')) {
      console.log('ACK', text.slice(4))
      return
    }
    if (text.startsWith('IP:')) {
      this.setData({ robotIp: text.slice(3) })
      return
    }
    if (text.startsWith('pwr:')) {
      this.setData({ battery: parseInt(text.slice(4), 10) })
      return
    }
    if (text.startsWith('fsm:')) {
      this.setData({ fsmState: parseInt(text.slice(4), 10) })
      return
    }
    console.log('notify', text)
  },

  async sendCommand(text) {
    if (Date.now() - this._lastCmdAt < CMD_COOLDOWN_MS) return
    this._lastCmdAt = Date.now()
    await this._write(text, true)
  },

  // --- 模式（进入遥控页先发 M_default）---
  sendModeDefault() { return this.sendCommand('M_default') },
  sendModeInit() { return this.sendCommand('M_init') },
  sendModeProtect() { return this.sendCommand('M_protect') },
  sendModeResetzero() { return this.sendCommand('M_resetzero') },
  sendModeTech() { return this.sendCommand('M_tech') },

  // --- 组合键（大小写与固件一致）---
  sendStand() { return this.sendCommand('LT+RT+start') },
  sendCrouch() { return this.sendCommand('LT+RT+RB') },
  /** 挥双手：仅单击触发一次，勿在 touchstart/touchend 各发一遍 */
  sendCheer() {
    const now = Date.now()
    if (now - this._lastCheerAt < CHEER_COOLDOWN_MS) return
    this._lastCheerAt = now
    return this.sendCommand('RT+A')
  },
  sendGaitToggle() { return this.sendCommand('LT+RT+LB') },
  sendUnload() { return this.sendCommand('LT+RT+B') },

  /** 左摇杆 detail.x/y；右摇杆 detail.x→rz（formatStick 内转协议 X/Y/Z） */
  onLeftStick(e) {
    const x = Number(e.detail.x || 0) / 100
    const y = Number(e.detail.y || 0) / 100
    this.setData({ lx: x, ly: y })
  },

  onRightStick(e) {
    const z = Number(e.detail.x || 0) / 100
    this.setData({ rz: z })
  },
})
