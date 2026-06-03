/**
 * Unified App Controller
 * 统一的应用控制器 - 群控和自控共用
 * 继承 BaseController，使用 WebSocket 通信
 */

class AppController extends BaseController {
    constructor(options = {}) {
        super();

        // Options
        this.hasModeSwitcher = options.hasModeSwitcher || false;
        this.modeSwitcher = null;

        // Commands configuration
        this.commandsList = [];
        this.categories = {};

        // Initialize Socket.IO
        this.socket = io();
        this.initSocketIO();

        // Initialize UI
        this.initVirtualJoysticks();
        this.loadCommands();
        this.initGamepad();

        // Start update loop
        this.startUpdateLoop();
    }

    initSocketIO() {
        this.socket.on('connect', () => {
            console.log('WebSocket connected');
            this.updateConnectionStatus(true);
        });

        this.socket.on('disconnect', () => {
            console.log('WebSocket disconnected');
            this.updateConnectionStatus(false);
        });

        this.socket.on('connection_status', (data) => {
            console.log('Connection status:', data);
        });
    }

    /**
     * 实现 BaseController 的抽象方法：发送摇杆数据
     */
    _doSendJoystickData() {
        // Don't send if in group mode (robot only)
        if (this.hasModeSwitcher && this.modeSwitcher && this.modeSwitcher.isGroupMode()) {
            return;
        }

        this.socket.emit('joystick', {
            left_x: this.leftX,
            left_y: this.leftY,
            right_x: this.rightX,
            right_y: this.rightY
        });
    }

    /**
     * 实现 BaseController 的抽象方法：发送头部控制数据
     */
    _doSendHeadControl() {
        // Don't send if in group mode (robot only)
        if (this.hasModeSwitcher && this.modeSwitcher && this.modeSwitcher.isGroupMode()) {
            return;
        }

        this.socket.emit('head_control', {
            yaw: this.headYaw,
            pitch: this.headPitch
        });
    }

    updateConnectionStatus(connected) {
        const statusEl = document.getElementById('connection-status');
        if (statusEl) {
            if (connected) {
                statusEl.textContent = '已连接';
                statusEl.className = 'status connected';
            } else {
                statusEl.textContent = '未连接';
                statusEl.className = 'status disconnected';
            }
        }
    }

    async loadCommands() {
        try {
            const response = await fetch('/api/commands');
            const data = await response.json();

            this.commandsList = data.commands || [];
            this.categories = data.categories || {};

            console.log('Loaded commands:', this.commandsList.length);
            console.log('Loaded categories:', this.categories);

            this.renderCommands();
        } catch (error) {
            console.error('Failed to load commands:', error);
            const container = document.getElementById('commands-container');
            if (container) {
                container.innerHTML = '<div class="error">命令加载失败</div>';
            }
        }
    }

    renderCommands() {
        const container = document.getElementById('commands-container');
        if (!container) return;

        container.innerHTML = '';

        // Sort categories by order
        const sortedCategories = Object.entries(this.categories)
            .sort((a, b) => (a[1].order || 0) - (b[1].order || 0));

        // Create command groups
        for (const [categoryId, categoryInfo] of sortedCategories) {
            const group = document.createElement('div');
            group.className = 'command-group';

            const title = document.createElement('h3');
            title.textContent = categoryInfo.title;
            group.appendChild(title);

            const buttonGrid = document.createElement('div');
            buttonGrid.className = 'button-grid';

            // Add buttons for this category
            for (const cmd of this.commandsList) {
                if (cmd.category === categoryId) {
                    const button = document.createElement('button');
                    button.className = 'cmd-btn';
                    button.setAttribute('data-command', cmd.code);
                    button.textContent = `${cmd.name} (${cmd.descript})`;
                    buttonGrid.appendChild(button);
                }
            }

            group.appendChild(buttonGrid);
            container.appendChild(group);
        }

        // Initialize button events after rendering
        this.initButtons();
    }

    initButtons() {
        // Add click handlers to all command buttons with debouncing
        const buttons = document.querySelectorAll('.cmd-btn');
        const buttonCooldown = new Map();
        const cooldownTime = 500; // 500ms cooldown

        buttons.forEach(button => {
            button.addEventListener('click', () => {
                // Check if in group mode (robot only)
                if (this.hasModeSwitcher && this.modeSwitcher && this.modeSwitcher.isGroupMode()) {
                    return;
                }

                const command = button.getAttribute('data-command');
                const now = Date.now();

                // Check cooldown
                if (buttonCooldown.has(command)) {
                    const lastClick = buttonCooldown.get(command);
                    if (now - lastClick < cooldownTime) {
                        console.log(`Command ${command} is on cooldown`);
                        return;
                    }
                }

                // Update cooldown
                buttonCooldown.set(command, now);

                // Send command
                this.sendCommand(command);

                // Visual feedback
                button.classList.add('active');
                setTimeout(() => {
                    button.classList.remove('active');
                }, 200);
            });
        });
    }

    initGamepad() {
        window.addEventListener('gamepadconnected', (e) => {
            console.log('Gamepad connected:', e.gamepad.id);
            this.gamepadIndex = e.gamepad.index;
            this.gamepadConnected = true;
            this.updateGamepadStatus(true, e.gamepad.id);
        });

        window.addEventListener('gamepaddisconnected', (e) => {
            console.log('Gamepad disconnected');
            if (e.gamepad.index === this.gamepadIndex) {
                this.gamepadConnected = false;
                this.gamepadIndex = null;
                this.updateGamepadStatus(false);
            }
        });

        // Check for already connected gamepads
        const gamepads = navigator.getGamepads();
        for (let i = 0; i < gamepads.length; i++) {
            if (gamepads[i]) {
                this.gamepadIndex = i;
                this.gamepadConnected = true;
                this.updateGamepadStatus(true, gamepads[i].id);
                break;
            }
        }
    }

    sendCommand(command) {
        console.log('Sending command:', command);
        this.socket.emit('command', { command: command });
    }
}

// Auto-hide landscape hint
function autoHideLandscapeHint() {
    const hint = document.getElementById('landscape-hint');
    if (hint && window.getComputedStyle(hint).display !== 'none') {
        setTimeout(() => {
            hint.classList.add('hiding');
            setTimeout(() => {
                hint.style.display = 'none';
            }, 500);
        }, 3000);
    }
}
