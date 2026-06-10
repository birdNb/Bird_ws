/**
 * 微信小程序 BLE 测试（重要：按服务 UUID 扫描，不要只按名称）
 *
 * app.json 需声明蓝牙权限；手机蓝牙 + 定位(GPS) 都要开。
 * 板子上必须先运行: ./start.sh  （脚本退出后小程序扫不到）
 */
const SERVICE_UUID = '0000FFF0-0000-1000-8000-00805F9B34FB'
const WRITE_UUID = '0000FFF1-0000-1000-8000-00805F9B34FB'
const NOTIFY_UUID = '0000FFF2-0000-1000-8000-00805F9B34FB'
const TARGET_NAME = 'Bird_BLE_Test'
// 板子蓝牙 MAC（与 ble_check.sh / start.sh 终端输出一致，按实际修改）
const TARGET_MAC_PREFIX = '00:19:86'

function normUuid(u) {
  return (u || '').replace(/-/g, '').toLowerCase()
}

function hasTargetService(device) {
  const list = device.advertisServiceUUIDs || []
  return list.some((u) => {
    const n = normUuid(u)
    return n === '0000fff0' || n === 'fff0' || n.includes('fff0')
  })
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

Page({
  data: { deviceId: '', log: '' },

  log(msg) {
    const line = `[${new Date().toLocaleTimeString()}] ${msg}`
    console.log(line)
    this.setData({ log: this.data.log + line + '\n' })
  },

  openAdapter() {
    wx.openBluetoothAdapter({
      mode: 'central',
      success: () => { this.log('蓝牙适配器已打开'); this.startScan() },
      fail: (e) => this.log('openBluetoothAdapter 失败: ' + JSON.stringify(e)),
    })
  },

  startScan() {
    // 关键：按 FFF0 服务 UUID 过滤，比只按名称可靠得多
    wx.startBluetoothDevicesDiscovery({
      services: [SERVICE_UUID],
      allowDuplicatesKey: true,
      powerLevel: 'high',
      success: () => {
        this.log('按服务 UUID 扫描: ' + SERVICE_UUID)
        this.log('（EDIFIER BLE 等其它设备会被过滤掉）')
        wx.onBluetoothDeviceFound((res) => {
          res.devices.forEach((d) => {
            const hit = nameMatch(d) || macMatch(d) || hasTargetService(d)
            this.log(
              '发现: id=' + d.deviceId +
              ' name=' + (d.name || '-') +
              ' localName=' + (d.localName || '-') +
              ' RSSI=' + d.RSSI +
              ' svcs=' + JSON.stringify(d.advertisServiceUUIDs || [])
            )
            if (hit && !this.data.deviceId) {
              this.log('>>> 匹配板子，开始连接')
              this.setData({ deviceId: d.deviceId })
              wx.stopBluetoothDevicesDiscovery({})
              this.connect(d.deviceId)
            }
          })
        })
        setTimeout(() => {
          if (!this.data.deviceId) {
            this.log('10s 内未找到板子: 确认板子 ./start.sh 在运行，且靠近板子')
          }
        }, 10000)
      },
      fail: (e) => this.log('扫描失败: ' + JSON.stringify(e)),
    })
  },

  connect(deviceId) {
    wx.createBLEConnection({
      deviceId,
      timeout: 10000,
      success: () => {
        this.log('已连接 ' + deviceId)
        wx.getBLEDeviceServices({
          deviceId,
          success: (r) => {
            this.log('服务: ' + r.services.map((s) => s.uuid).join(', '))
            const sid = r.services.find((s) => normUuid(s.uuid).includes('fff0'))
            const serviceId = sid ? sid.uuid : SERVICE_UUID
            wx.getBLEDeviceCharacteristics({
              deviceId,
              serviceId,
              success: (c) => {
                this.log('特征: ' + c.characteristics.map((x) => x.uuid).join(', '))
                wx.notifyBLECharacteristicValueChange({
                  deviceId,
                  serviceId,
                  characteristicId: NOTIFY_UUID,
                  state: true,
                  success: () => this.log('已订阅 notify'),
                })
                wx.onBLECharacteristicValueChange((ev) => {
                  this.log('收到 notify: ' + ab2str(ev.value))
                })
              },
            })
          },
        })
      },
      fail: (e) => this.log('连接失败: ' + JSON.stringify(e)),
    })
  },

  sendTest() {
    const deviceId = this.data.deviceId
    if (!deviceId) { this.log('请先扫描连接'); return }
    const text = 'hello from miniprogram ' + Date.now()
    const buffer = new ArrayBuffer(text.length)
    const view = new Uint8Array(buffer)
    for (let i = 0; i < text.length; i++) view[i] = text.charCodeAt(i)
    wx.writeBLECharacteristicValue({
      deviceId,
      serviceId: SERVICE_UUID,
      characteristicId: WRITE_UUID,
      value: buffer,
      success: () => this.log('已发送: ' + text),
      fail: (e) => this.log('发送失败: ' + JSON.stringify(e)),
    })
  },
})
