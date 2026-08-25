/**
 * Bird BLE 指令参考 — 与 BLE_PROTOCOL.md 完全一致
 * FFE0/FFE1/FFE2 | 摇杆 20Hz | 可选 ,N:序号 保活
 */
const SERVICE_UUID = '0000FFE0-0000-1000-8000-00805F9B34FB'
const WRITE_UUID = '0000FFE1-0000-1000-8000-00805F9B34FB'
const NOTIFY_UUID = '0000FFE2-0000-1000-8000-00805F9B34FB'
const TARGET_NAME = 'HT_88888888' // 可被 rename HT_12345678 修改
const STICK_INTERVAL_MS = 50 // 20Hz
const STICK_DEADZONE = 10 // UI -100~100 刻度
const STICK_XY_SCALE = 1.8 // 前后/左右满量程 ±1.8
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
  let x = applyDeadzone(-ly * 100) * STICK_XY_SCALE
  let y = applyDeadzone(lx * 100) * STICK_XY_SCALE
  x = Math.max(-STICK_XY_SCALE, Math.min(STICK_XY_SCALE, x))
  y = Math.max(-STICK_XY_SCALE, Math.min(STICK_XY_SCALE, y))
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
    // 主广播含 128-bit FFE0，Scan Response 含设备名。
    // 安卓勿用 services 过滤（微信部分机型仍漏扫）；统一全量扫再按名/UUID 过滤。
    const opts = {
      allowDuplicatesKey: true,
      powerLevel: 'high',
      success: () => {
        wx.onBluetoothDeviceFound((res) => {
          res.devices.forEach((d) => {
            if (this.data.deviceId) return
            const name = (d.name || d.localName || '').toLowerCase()
            const uuids = d.advertisServiceUUIDs || []
            const serviceHit = uuids.some((u) => uuidHit(u, 'ffe0'))
            const nameHit = name.includes('ht_') || name.includes('bird_ble')
              || name === TARGET_NAME.toLowerCase()
            if (!nameHit && !serviceHit) return
            this.setData({ deviceId: d.deviceId })
            wx.stopBluetoothDevicesDiscovery({})
            this.establishBleLink(d.deviceId)
          })
        })
      },
    }
    wx.startBluetoothDevicesDiscovery(opts)
  },

  async establishBleLink(deviceId) {
    await this._createConnection(deviceId)
    await delay(isIOS() ? 2000 : 800)
    if (wx.setBLEMTU) wx.setBLEMTU({ deviceId, mtu: 247 })
    await this._discoverGatt(deviceId)
    await this._subscribeNotify(deviceId)
    await delay(isIOS() ? 350 : 150)
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
    // 步态/电源/语音：板端原样回传相同指令确认
    if (
      text === 'MP ON' || text === 'MP OFF' ||
      text === 'GAIT ON' || text === 'GAIT OFF' ||
      text === 'sound ON' || text === 'sound OFF'
    ) {
      console.log('CMD_ECHO', text)
      return
    }
    if (text.startsWith('rename HT_')) {
      console.log('BLE_RENAME', text)
      this.setData({ targetName: text.slice(7) })
      return
    }
    if (text.startsWith('ip:')) {
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

  /** 音量滑块松手后调用，percent 为 0–100 整数，例如 sendVolume(10) → V 10 */
  async sendVolume(percent) {
    const pct = Math.max(0, Math.min(100, Math.round(Number(percent) || 0)))
    await this.sendCommand(`V ${pct}`)
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
  /** 握手（RT+Y 短脉冲，模拟手柄） */
  sendHandshake() { return this.sendCommand('RT+Y') },
  /** 摇手防守（RT+B 短脉冲，模拟手柄） */
  sendWaveDefense() { return this.sendCommand('RT+B') },
  sendGaitOn() { return this.sendCommand('GAIT ON') },
  sendGaitOff() { return this.sendCommand('GAIT OFF') },
  /** 拉动 pull_move 控制：开启/关闭 torque-cmd-vel.service */
  sendPullOn() { return this.sendCommand('PULL ON') },
  sendPullOff() { return this.sendCommand('PULL OFF') },
  /** 修改 BLE 广播名后8位，例如 sendRename('12345678') */
  sendRename(digits8) {
    const d = String(digits8 || '').replace(/\D/g, '').slice(-8).padStart(8, '0')
    return this.sendCommand(`rename HT_${d}`)
  },
  sendUnload() { return this.sendCommand('LT+RT+B') },

  /** 疾跑开关：按住策略侧 LT 加速（AMP Soccer 模式） */
  sendSprintOn() { return this.sendCommand('LT ON') },
  sendSprintOff() { return this.sendCommand('LT OFF') },

  /** 对话语音：录音文案前5字拼音首字母大写，如 sendConversation('LYJXD') */
  sendConversation(code) {
    return this.sendCommand(String(code || '').trim().toUpperCase())
  },

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
