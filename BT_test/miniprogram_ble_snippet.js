/**
 * 微信小程序 BLE 控制参考（板子 ./start.sh 须常开）
 *
 * 关键：createBLEConnection 只建立链路，必须 write 到 FFF1 板子才会打印/控机器人。
 * app.json 需声明蓝牙权限；手机蓝牙 + 定位(GPS) 都要开。
 */
const SERVICE_UUID = '0000FFF0-0000-1000-8000-00805F9B34FB'
const WRITE_UUID = '0000FFF1-0000-1000-8000-00805F9B34FB'
const NOTIFY_UUID = '0000FFF2-0000-1000-8000-00805F9B34FB'
const TARGET_NAME = 'Bird_BLE_Test'
const TARGET_MAC_PREFIX = '00:19:86'

function normUuid(u) {
  return (u || '').replace(/-/g, '').toLowerCase()
}

function uuidHit(u, needle) {
  const n = normUuid(u)
  return n === needle || n.includes(needle)
}

function hasTargetService(device) {
  const list = device.advertisServiceUUIDs || []
  return list.some((u) => uuidHit(u, 'fff0'))
}

function nameMatch(device) {
  const n = (device.name || device.localName || '').toLowerCase()
  return n.includes('bird_ble') || n === TARGET_NAME.toLowerCase()
}

function macMatch(device) {
  const id = (device.deviceId || '').toUpperCase()
  return id.includes(TARGET_MAC_PREFIX.toUpperCase())
}

function ab2str(buffer) {
  const arr = new Uint8Array(buffer)
  let s = ''
  for (let i = 0; i < arr.length; i++) s += String.fromCharCode(arr[i])
  try { return decodeURIComponent(escape(s)) } catch (e) { return s }
}

function str2ab(text) {
  const buffer = new ArrayBuffer(text.length)
  const view = new Uint8Array(buffer)
  for (let i = 0; i < text.length; i++) view[i] = text.charCodeAt(i)
  return buffer
}

Page({
  data: {
    deviceId: '',
    log: '',
    lx: 0,
    ly: 0,
    rz: 0,
    gattReady: false,
  },

  // 连接后由 getBLEDeviceCharacteristics 填充（勿硬编码，用发现到的 UUID）
  _serviceId: '',
  _writeCharId: '',
  _notifyCharId: '',
  _sendTimer: null,

  log(msg) {
    const line = `[${new Date().toLocaleTimeString()}] ${msg}`
    console.log(line)
    this.setData({ log: this.data.log + line + '\n' })
  },

  onUnload() {
    this.stopSendLoop()
    if (this.data.deviceId) {
      wx.closeBLEConnection({ deviceId: this.data.deviceId })
    }
  },

  openAdapter() {
    wx.openBluetoothAdapter({
      mode: 'central',
      success: () => { this.log('蓝牙适配器已打开'); this.startScan() },
      fail: (e) => this.log('openBluetoothAdapter 失败: ' + JSON.stringify(e)),
    })
  },

  startScan() {
    wx.startBluetoothDevicesDiscovery({
      services: [SERVICE_UUID],
      allowDuplicatesKey: true,
      powerLevel: 'high',
      success: () => {
        this.log('按服务 UUID 扫描: ' + SERVICE_UUID)
        wx.onBluetoothDeviceFound((res) => {
          res.devices.forEach((d) => {
            const hit = nameMatch(d) || macMatch(d) || hasTargetService(d)
            if (hit && !this.data.deviceId) {
              this.log('>>> 匹配板子，开始连接')
              this.setData({ deviceId: d.deviceId })
              wx.stopBluetoothDevicesDiscovery({})
              this.connect(d.deviceId)
            }
          })
        })
      },
      fail: (e) => this.log('扫描失败: ' + JSON.stringify(e)),
    })
  },

  connect(deviceId) {
    this.setData({ gattReady: false })
    wx.createBLEConnection({
      deviceId,
      timeout: 10000,
      success: () => {
        this.log('BLE 链路已连接，正在发现 GATT 服务...')
        if (wx.setBLEMTU) {
          wx.setBLEMTU({ deviceId, mtu: 512, fail: () => {} })
        }
        // 微信建议连接后稍等再 discover
        setTimeout(() => this.discoverGatt(deviceId), 400)
      },
      fail: (e) => this.log('连接失败: ' + JSON.stringify(e)),
    })
  },

  discoverGatt(deviceId) {
    wx.getBLEDeviceServices({
      deviceId,
      success: (r) => {
        this.log('服务: ' + r.services.map((s) => s.uuid).join(', '))
        const sid = r.services.find((s) => uuidHit(s.uuid, 'fff0'))
        if (!sid) {
          this.log('未找到 FFF0 服务')
          return
        }
        this._serviceId = sid.uuid
        wx.getBLEDeviceCharacteristics({
          deviceId,
          serviceId: this._serviceId,
          success: (c) => {
            this.log('特征: ' + c.characteristics.map((x) => x.uuid).join(', '))
            const writeCh = c.characteristics.find((x) => uuidHit(x.uuid, 'fff1'))
            const notifyCh = c.characteristics.find((x) => uuidHit(x.uuid, 'fff2'))
            if (!writeCh) {
              this.log('未找到 FFF1 可写特征')
              return
            }
            this._writeCharId = writeCh.uuid
            this._writeNoResponse = !!(writeCh.properties && writeCh.properties.writeNoResponse)
            this.setData({ gattReady: true })
            this.log('GATT 就绪，可写入 FFF1')

            if (notifyCh) {
              this._notifyCharId = notifyCh.uuid
              wx.notifyBLECharacteristicValueChange({
                deviceId,
                serviceId: this._serviceId,
                characteristicId: this._notifyCharId,
                state: true,
                success: () => this.log('已订阅 notify (FFF2)'),
              })
              wx.onBLECharacteristicValueChange((ev) => {
                this.log('收到 notify: ' + ab2str(ev.value))
              })
            }

            // 连接后立刻发一条，板子终端应出现 >>> 收到手机消息
            this.sendText('X:0.00,Y:0.00,Z:0.00')
            this.startSendLoop()
          },
          fail: (e) => this.log('getBLEDeviceCharacteristics 失败: ' + JSON.stringify(e)),
        })
      },
      fail: (e) => this.log('getBLEDeviceServices 失败: ' + JSON.stringify(e)),
    })
  },

  sendText(text) {
    const deviceId = this.data.deviceId
    if (!deviceId || !this._serviceId || !this._writeCharId) {
      this.log('GATT 未就绪，无法发送')
      return
    }
    const req = {
      deviceId,
      serviceId: this._serviceId,
      characteristicId: this._writeCharId,
      value: str2ab(text),
      success: () => {},
      fail: (e) => this.log('写入失败: ' + JSON.stringify(e)),
    }
    if (this._writeNoResponse) {
      req.writeType = 'writeNoResponse'
    }
    wx.writeBLECharacteristicValue(req)
  },

  startSendLoop() {
    this.stopSendLoop()
    // 摇杆数据需周期性 write，板子才会持续收到
    this._sendTimer = setInterval(() => {
      const { lx, ly, rz } = this.data
      const text = `X:${lx.toFixed(2)},Y:${ly.toFixed(2)},Z:${rz.toFixed(2)}`
      this.sendText(text)
    }, 100)
  },

  stopSendLoop() {
    if (this._sendTimer) {
      clearInterval(this._sendTimer)
      this._sendTimer = null
    }
  },

  sendTest() {
    this.sendText('hello from miniprogram ' + Date.now())
  },

  sendStand() {
    this.sendText('LT+RT+START')
  },

  sendCrouch() {
    this.sendText('LT+RT+RB')
  },

  sendUnload() {
    this.sendText('LT+RT+B')
  },

  onStickChange(e) {
    const v = Number(e.detail.value) / 100
    const axis = e.currentTarget.dataset.axis
    if (axis === 'x') this.setData({ lx: v })
    if (axis === 'y') this.setData({ ly: v })
    if (axis === 'z') this.setData({ rz: v })
  },
})
