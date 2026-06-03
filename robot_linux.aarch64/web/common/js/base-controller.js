/**
 * Base Controller - 公共控制器基类
 * 提供摇杆、头部控制、按键等公共逻辑
 */

class BaseController {
    constructor() {
        // Joystick state
        this.leftX = 0;
        this.leftY = 0;
        this.rightX = 0;
        this.rightY = 0;

        // Head control state (yaw/pitch)
        this.headYaw = 0;
        this.headPitch = 0;

        // Physical gamepad state
        this.gamepadIndex = null;
        this.gamepadConnected = false;

        // Send rate limiting
        this.sendRate = 30; // Hz
        this.lastSendTime = 0;
        this.lastHeadSendTime = 0;

        // Track last sent values to avoid sending duplicates
        this.lastSentValues = {
            left_x: 0,
            left_y: 0,
            right_x: 0,
            right_y: 0
        };

        this.lastSentHeadValues = {
            yaw: 0,
            pitch: 0
        };
    }

    /**
     * 初始化虚拟摇杆（子类需要实现具体的发送逻辑）
     */
    initVirtualJoysticks() {
        // Left joystick
        this.leftJoystick = new VirtualJoystick('left-joystick', (x, y) => {
            this.leftX = x;
            this.leftY = y;
            document.getElementById('left-x').textContent = x.toFixed(2);
            document.getElementById('left-y').textContent = y.toFixed(2);

            // Immediately send data when joystick is released (returns to 0)
            if (x === 0 && y === 0) {
                this.sendJoystickDataImmediate();
            }
        });

        // Right joystick
        this.rightJoystick = new VirtualJoystick('right-joystick', (x, y) => {
            this.rightX = x;
            this.rightY = y;
            document.getElementById('right-x').textContent = x.toFixed(2);
            document.getElementById('right-y').textContent = y.toFixed(2);

            // Immediately send data when joystick is released (returns to 0)
            if (x === 0 && y === 0) {
                this.sendJoystickDataImmediate();
            }
        });

        // Head control joystick
        this.headJoystick = new VirtualJoystick('head-joystick', (x, y) => {
            this.headYaw = x;
            this.headPitch = y;
            document.getElementById('head-yaw').textContent = x.toFixed(2);
            document.getElementById('head-pitch').textContent = y.toFixed(2);

            // Immediately send data when joystick is released (returns to 0)
            if (x === 0 && y === 0) {
                this.sendHeadDataImmediate();
            }
        });
    }

    /**
     * 发送摇杆数据（带频率限制）
     * 子类需要实现具体的发送逻辑
     */
    sendJoystickData() {
        const now = Date.now();
        if (now - this.lastSendTime < 1000 / this.sendRate) {
            return;
        }

        // Check if any joystick value is non-zero OR changed from last sent
        const threshold = 0.01;
        const hasMovement = Math.abs(this.leftX) > threshold ||
                          Math.abs(this.leftY) > threshold ||
                          Math.abs(this.rightX) > threshold ||
                          Math.abs(this.rightY) > threshold;

        const hasChanged = Math.abs(this.leftX - this.lastSentValues.left_x) > threshold ||
                          Math.abs(this.leftY - this.lastSentValues.left_y) > threshold ||
                          Math.abs(this.rightX - this.lastSentValues.right_x) > threshold ||
                          Math.abs(this.rightY - this.lastSentValues.right_y) > threshold;

        // Only send if there's movement OR values changed (including returning to 0)
        if (hasMovement || hasChanged) {
            this._doSendJoystickData();

            // Update last sent values
            this.lastSentValues.left_x = this.leftX;
            this.lastSentValues.left_y = this.leftY;
            this.lastSentValues.right_x = this.rightX;
            this.lastSentValues.right_y = this.rightY;

            this.lastSendTime = now;
        }
    }

    /**
     * 立即发送摇杆数据（用于释放摇杆时）
     */
    sendJoystickDataImmediate() {
        // Send multiple times to ensure delivery (UDP is unreliable)
        const sendCount = 3;
        for (let i = 0; i < sendCount; i++) {
            this._doSendJoystickData();
        }

        // Update last sent values
        this.lastSentValues.left_x = this.leftX;
        this.lastSentValues.left_y = this.leftY;
        this.lastSentValues.right_x = this.rightX;
        this.lastSentValues.right_y = this.rightY;

        this.lastSendTime = Date.now();
    }

    /**
     * 发送头部控制数据（带频率限制）
     */
    sendHeadControl() {
        const now = Date.now();
        if (now - this.lastHeadSendTime < 1000 / this.sendRate) {
            return;
        }

        // Check if head position changed
        const threshold = 0.01;
        const hasMovement = Math.abs(this.headYaw) > threshold ||
                          Math.abs(this.headPitch) > threshold;

        const hasChanged = Math.abs(this.headYaw - this.lastSentHeadValues.yaw) > threshold ||
                          Math.abs(this.headPitch - this.lastSentHeadValues.pitch) > threshold;

        // Only send if there's movement OR values changed
        if (hasMovement || hasChanged) {
            this._doSendHeadControl();

            // Update last sent values
            this.lastSentHeadValues.yaw = this.headYaw;
            this.lastSentHeadValues.pitch = this.headPitch;

            this.lastHeadSendTime = now;
        }
    }

    /**
     * 立即发送头部控制数据（用于释放摇杆时）
     */
    sendHeadDataImmediate() {
        // Send multiple times to ensure delivery
        const sendCount = 3;
        for (let i = 0; i < sendCount; i++) {
            this._doSendHeadControl();
        }

        // Update last sent values
        this.lastSentHeadValues.yaw = this.headYaw;
        this.lastSentHeadValues.pitch = this.headPitch;

        this.lastHeadSendTime = Date.now();
    }

    /**
     * 实际发送摇杆数据的方法（子类必须实现）
     */
    _doSendJoystickData() {
        throw new Error('子类必须实现 _doSendJoystickData() 方法');
    }

    /**
     * 实际发送头部控制数据的方法（子类必须实现）
     */
    _doSendHeadControl() {
        throw new Error('子类必须实现 _doSendHeadControl() 方法');
    }

    /**
     * 更新物理手柄输入
     */
    updateGamepadInput() {
        if (!this.gamepadConnected) return;

        const gamepads = navigator.getGamepads();
        if (!gamepads[this.gamepadIndex]) return;

        const gamepad = gamepads[this.gamepadIndex];

        // Update joystick values from physical gamepad
        // Left stick: axes 0 (X), 1 (Y)
        // Right stick: axes 2 (X), 3 (Y)
        const deadzone = 0.1;

        const applyDeadzone = (value) => {
            return Math.abs(value) < deadzone ? 0 : value;
        };

        this.leftX = applyDeadzone(gamepad.axes[0] || 0);
        this.leftY = applyDeadzone(gamepad.axes[1] || 0);
        this.rightX = applyDeadzone(gamepad.axes[2] || 0);
        this.rightY = applyDeadzone(gamepad.axes[3] || 0);

        // Update display
        document.getElementById('left-x').textContent = this.leftX.toFixed(2);
        document.getElementById('left-y').textContent = this.leftY.toFixed(2);
        document.getElementById('right-x').textContent = this.rightX.toFixed(2);
        document.getElementById('right-y').textContent = this.rightY.toFixed(2);
    }

    /**
     * 启动更新循环
     */
    startUpdateLoop() {
        const update = () => {
            // Update gamepad input if connected
            if (this.gamepadConnected) {
                this.updateGamepadInput();
            }

            // Send joystick data
            this.sendJoystickData();

            // Send head control data
            this.sendHeadControl();

            // Continue loop
            requestAnimationFrame(update);
        };

        update();
    }

    /**
     * 更新手柄状态显示
     */
    updateGamepadStatus(connected, name = '') {
        const statusEl = document.getElementById('gamepad-text');
        if (connected) {
            statusEl.textContent = `已连接: ${name}`;
            statusEl.style.color = '#4caf50';
        } else {
            statusEl.textContent = '未检测到物理手柄';
            statusEl.style.color = '';
        }
    }
}
