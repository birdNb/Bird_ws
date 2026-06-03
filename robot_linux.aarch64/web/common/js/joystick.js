/**
 * Virtual Joystick Controller
 * Supports touch and mouse input
 */

class VirtualJoystick {
    constructor(canvasId, onUpdate) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.onUpdate = onUpdate;

        // Joystick state
        this.x = 0;
        this.y = 0;
        this.active = false;

        // Set canvas internal size to match CSS display size
        this.updateCanvasSize();

        // Bind events
        this.bindEvents();

        // Update size on window resize
        window.addEventListener('resize', () => {
            this.updateCanvasSize();
            this.draw();
        });

        // Start drawing
        this.draw();
    }

    updateCanvasSize() {
        // Get actual CSS display size
        const rect = this.canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;

        // Set canvas internal size to match display size (with retina support)
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;

        // Get fresh context and scale for retina displays
        // Note: setting width/height resets the context, so scale won't accumulate
        this.ctx = this.canvas.getContext('2d');
        this.ctx.scale(dpr, dpr);

        // Update dimensions (use CSS size, not internal canvas size)
        this.centerX = rect.width / 2;
        this.centerY = rect.height / 2;
        this.maxRadius = Math.min(rect.width, rect.height) / 2 - 20;
        this.stickRadius = 15;
    }

    bindEvents() {
        // Mouse events
        this.canvas.addEventListener('mousedown', (e) => this.onPointerDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.onPointerMove(e));
        this.canvas.addEventListener('mouseup', () => this.onPointerUp());
        this.canvas.addEventListener('mouseleave', () => this.onPointerUp());

        // Touch events
        this.canvas.addEventListener('touchstart', (e) => {
            e.preventDefault();
            // Use targetTouches to get only touches on this canvas
            if (e.targetTouches.length > 0) {
                this.onPointerDown(e.targetTouches[0]);
            }
        });
        this.canvas.addEventListener('touchmove', (e) => {
            e.preventDefault();
            // Use targetTouches to avoid interference from other touches
            if (e.targetTouches.length > 0) {
                this.onPointerMove(e.targetTouches[0]);
            }
        });
        this.canvas.addEventListener('touchend', (e) => {
            e.preventDefault();
            this.onPointerUp();
        });
    }

    onPointerDown(e) {
        // Recalibrate size on each touch (fixes mobile viewport changes)
        this.updateCanvasSize();
        this.active = true;
        this.updatePosition(e);
    }

    onPointerMove(e) {
        if (this.active) {
            this.updatePosition(e);
        }
    }

    onPointerUp() {
        this.active = false;
        this.x = 0;
        this.y = 0;
        this.draw();
        this.onUpdate(0, 0);
    }

    updatePosition(e) {
        const rect = this.canvas.getBoundingClientRect();
        const clientX = e.clientX - rect.left;
        const clientY = e.clientY - rect.top;

        // Calculate relative position
        let dx = clientX - this.centerX;
        let dy = clientY - this.centerY;

        // Limit to max radius
        const distance = Math.sqrt(dx * dx + dy * dy);
        if (distance > this.maxRadius) {
            const angle = Math.atan2(dy, dx);
            dx = Math.cos(angle) * this.maxRadius;
            dy = Math.sin(angle) * this.maxRadius;
        }

        // Normalize to -1.0 to 1.0
        this.x = dx / this.maxRadius;
        this.y = dy / this.maxRadius;

        this.draw();
        this.onUpdate(this.x, this.y);
    }

    draw() {
        // Clear canvas
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // Draw background circle
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, this.maxRadius, 0, Math.PI * 2);
        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.1)';
        this.ctx.fill();
        this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // Draw crosshair
        this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        this.ctx.lineWidth = 1;
        this.ctx.beginPath();
        this.ctx.moveTo(this.centerX - this.maxRadius, this.centerY);
        this.ctx.lineTo(this.centerX + this.maxRadius, this.centerY);
        this.ctx.moveTo(this.centerX, this.centerY - this.maxRadius);
        this.ctx.lineTo(this.centerX, this.centerY + this.maxRadius);
        this.ctx.stroke();

        // Draw joystick position
        const stickX = this.centerX + this.x * this.maxRadius;
        const stickY = this.centerY + this.y * this.maxRadius;

        // Draw stick
        this.ctx.beginPath();
        this.ctx.arc(stickX, stickY, this.stickRadius, 0, Math.PI * 2);
        this.ctx.fillStyle = this.active ? 'rgba(100, 200, 255, 0.9)' : 'rgba(100, 200, 255, 0.7)';
        this.ctx.fill();
        this.ctx.strokeStyle = '#fff';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // Draw center dot
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, 3, 0, Math.PI * 2);
        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
        this.ctx.fill();
    }

    reset() {
        this.x = 0;
        this.y = 0;
        this.active = false;
        this.draw();
    }
}

// Export for use in main app
window.VirtualJoystick = VirtualJoystick;
