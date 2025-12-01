/**
 * Path Planning Panel
 * Pattern test and path execution control
 */

// State
let patternServiceRunning = false;
let vizSystemRunning = false;
let testRunning = false;
let currentPosition = { x: 0, y: 0, z: 0 };
let testStartTime = null;
let elapsedInterval = null;

// Pattern definitions for preview
const PATTERNS = {
    diamond: { points: 4, shape: 'diamond' },
    square: { points: 4, shape: 'square' },
    circle: { points: 36, shape: 'circle' },
    spiral: { points: 50, shape: 'spiral' },
    figure8: { points: 40, shape: 'figure8' },
    triangle: { points: 3, shape: 'triangle' },
    star: { points: 5, shape: 'star' }
};

// ==================== Initialization ====================

document.addEventListener('DOMContentLoaded', async () => {
    console.log('[PathPlanning] Initializing...');

    // Load robot/server info from session
    loadSessionInfo();

    // Init sliders
    initSliders();

    // Init pattern preview
    initPatternPreview();

    // Setup event handlers
    setupEventHandlers();

    // Periodic updates
    setInterval(updateTimestamp, 1000);

    console.log('[PathPlanning] Initialized');
});

function loadSessionInfo() {
    const robot = sessionStorage.getItem('selectedRobot') || '-';
    const server = sessionStorage.getItem('selectedServer') || '-';

    document.getElementById('header-robot').textContent = robot;
    document.getElementById('header-server').textContent = server;
}

// ==================== Sliders ====================

function initSliders() {
    const sizeSlider = document.getElementById('size-slider');
    const speedSlider = document.getElementById('speed-slider');

    sizeSlider.addEventListener('input', () => {
        const value = parseFloat(sizeSlider.value).toFixed(2);
        document.getElementById('size-value').textContent = value;
        document.getElementById('preview-size').textContent = value;
        drawPatternPreview();
    });

    speedSlider.addEventListener('input', () => {
        document.getElementById('speed-value').textContent = `${parseFloat(speedSlider.value).toFixed(1)}x`;
    });
}

// ==================== Pattern Preview ====================

function initPatternPreview() {
    const patternSelect = document.getElementById('pattern-select');
    patternSelect.addEventListener('change', () => {
        document.getElementById('preview-pattern').textContent = patternSelect.value;
        drawPatternPreview();
    });

    // Initial draw
    drawPatternPreview();
}

function drawPatternPreview() {
    const canvas = document.getElementById('pattern-canvas');
    const ctx = canvas.getContext('2d');
    const pattern = document.getElementById('pattern-select').value;
    const size = parseFloat(document.getElementById('size-slider').value);

    // Clear canvas
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw grid
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 0.5;
    const gridSize = 30;
    for (let x = 0; x <= canvas.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
    }
    for (let y = 0; y <= canvas.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }

    // Draw center crosshair
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - 10, cy);
    ctx.lineTo(cx + 10, cy);
    ctx.moveTo(cx, cy - 10);
    ctx.lineTo(cx, cy + 10);
    ctx.stroke();

    // Calculate scale (1m = ~200px at size 0.5)
    const scale = 400 * size;
    const points = generatePatternPoints(pattern, scale);

    // Draw pattern
    if (points.length > 0) {
        ctx.strokeStyle = '#4caf50';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cx + points[0].x, cy + points[0].y);
        for (let i = 1; i < points.length; i++) {
            ctx.lineTo(cx + points[i].x, cy + points[i].y);
        }
        ctx.closePath();
        ctx.stroke();

        // Draw points
        ctx.fillStyle = '#2196f3';
        for (const point of points) {
            ctx.beginPath();
            ctx.arc(cx + point.x, cy + point.y, 3, 0, Math.PI * 2);
            ctx.fill();
        }

        // Draw start point (larger)
        ctx.fillStyle = '#f44336';
        ctx.beginPath();
        ctx.arc(cx + points[0].x, cy + points[0].y, 5, 0, Math.PI * 2);
        ctx.fill();
    }

    // Update point count
    document.getElementById('preview-points').textContent = points.length;
}

function generatePatternPoints(pattern, scale) {
    const points = [];
    const halfScale = scale / 2;

    switch (pattern) {
        case 'diamond':
            points.push({ x: 0, y: -halfScale });
            points.push({ x: halfScale, y: 0 });
            points.push({ x: 0, y: halfScale });
            points.push({ x: -halfScale, y: 0 });
            break;

        case 'square':
            points.push({ x: -halfScale, y: -halfScale });
            points.push({ x: halfScale, y: -halfScale });
            points.push({ x: halfScale, y: halfScale });
            points.push({ x: -halfScale, y: halfScale });
            break;

        case 'circle':
            for (let i = 0; i < 36; i++) {
                const angle = (i / 36) * Math.PI * 2;
                points.push({
                    x: Math.cos(angle) * halfScale,
                    y: Math.sin(angle) * halfScale
                });
            }
            break;

        case 'triangle':
            for (let i = 0; i < 3; i++) {
                const angle = (i / 3) * Math.PI * 2 - Math.PI / 2;
                points.push({
                    x: Math.cos(angle) * halfScale,
                    y: Math.sin(angle) * halfScale
                });
            }
            break;

        case 'star':
            for (let i = 0; i < 10; i++) {
                const angle = (i / 10) * Math.PI * 2 - Math.PI / 2;
                const r = i % 2 === 0 ? halfScale : halfScale * 0.4;
                points.push({
                    x: Math.cos(angle) * r,
                    y: Math.sin(angle) * r
                });
            }
            break;

        case 'figure8':
            for (let i = 0; i < 40; i++) {
                const t = (i / 40) * Math.PI * 2;
                points.push({
                    x: Math.sin(t) * halfScale,
                    y: Math.sin(2 * t) * halfScale * 0.5
                });
            }
            break;

        case 'spiral':
            for (let i = 0; i < 50; i++) {
                const t = (i / 50) * Math.PI * 4;
                const r = (i / 50) * halfScale;
                points.push({
                    x: Math.cos(t) * r,
                    y: Math.sin(t) * r
                });
            }
            break;
    }

    return points;
}

// ==================== Event Handlers ====================

function setupEventHandlers() {
    // Service control
    document.getElementById('btn-pattern-service').addEventListener('click', togglePatternService);
    document.getElementById('btn-viz-system').addEventListener('click', toggleVizSystem);

    // Path execution
    document.getElementById('btn-start-path').addEventListener('click', startPath);
    document.getElementById('btn-stop-path').addEventListener('click', stopPath);

    // Pattern test
    document.getElementById('btn-preview').addEventListener('click', previewTest);
    document.getElementById('btn-start-test').addEventListener('click', startTest);
    document.getElementById('btn-stop-test').addEventListener('click', stopTest);

    // Joint control
    document.getElementById('btn-home').addEventListener('click', moveToHome);
    document.getElementById('btn-zero').addEventListener('click', moveToZero);

    // Output controls
    document.getElementById('btn-clear-output').addEventListener('click', clearOutput);
    document.getElementById('btn-copy-output').addEventListener('click', copyOutput);
}

// ==================== Service Control ====================

async function togglePatternService() {
    const btn = document.getElementById('btn-pattern-service');
    const action = patternServiceRunning ? 'stop' : 'start';

    try {
        log(`${action === 'start' ? 'Starting' : 'Stopping'} pattern service...`);

        const params = {
            pattern: document.getElementById('pattern-select').value,
            size: parseFloat(document.getElementById('size-slider').value),
            speed: parseFloat(document.getElementById('speed-slider').value)
        };

        const result = await api.togglePatternService(action, params);

        if (result.status === 'started' || result.status === 'success') {
            patternServiceRunning = true;
            btn.textContent = 'Stop Service';
            btn.classList.replace('btn-primary', 'btn-danger');
            updateStatus('status-pattern-service', 'Running', 'running');
            enableTestButtons(true);
            log('Pattern service started');
        } else if (result.status === 'stopped') {
            patternServiceRunning = false;
            btn.textContent = 'Launch Service';
            btn.classList.replace('btn-danger', 'btn-primary');
            updateStatus('status-pattern-service', 'Stopped', 'stopped');
            enableTestButtons(false);
            log('Pattern service stopped');
        }
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

async function toggleVizSystem() {
    const btn = document.getElementById('btn-viz-system');
    const action = vizSystemRunning ? 'stop' : 'start';

    try {
        log(`${action === 'start' ? 'Starting' : 'Stopping'} visualization system...`);
        const result = await api.toggleVizSystem(action);

        if (result.status === 'started' || result.status === 'success') {
            vizSystemRunning = true;
            btn.textContent = 'Stop Viz';
            btn.classList.replace('btn-secondary', 'btn-danger');
            log('Visualization system started');
        } else if (result.status === 'stopped') {
            vizSystemRunning = false;
            btn.textContent = 'Visualization System';
            btn.classList.replace('btn-danger', 'btn-secondary');
            log('Visualization system stopped');
        }
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

// ==================== Path Execution ====================

async function startPath() {
    try {
        log('Starting path execution...');
        const result = await api.startPathPlanning();
        if (result.status === 'started' || result.status === 'success') {
            document.getElementById('btn-start-path').disabled = true;
            document.getElementById('btn-stop-path').disabled = false;
            log('Path execution started');
        }
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

async function stopPath() {
    try {
        log('Stopping path execution...');
        const result = await api.stopPathPlanning();
        if (result.status === 'stopped' || result.status === 'success') {
            document.getElementById('btn-start-path').disabled = false;
            document.getElementById('btn-stop-path').disabled = true;
            log('Path execution stopped');
        }
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

// ==================== Pattern Test ====================

async function previewTest() {
    try {
        log('Previewing pattern...');
        const result = await api.previewPatternTest();
        log(`Preview: ${JSON.stringify(result)}`);
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

async function startTest() {
    try {
        log('Starting pattern test...');
        const result = await api.startPatternTest();

        if (result.status === 'started' || result.status === 'success') {
            testRunning = true;
            testStartTime = Date.now();
            updateStatus('status-test', 'Running', 'running');
            document.getElementById('btn-start-test').disabled = true;
            document.getElementById('btn-stop-test').disabled = false;
            startElapsedTimer();
            log('Pattern test started');
        }
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

async function stopTest() {
    try {
        log('Stopping pattern test...');
        const result = await api.stopPatternTest();

        if (result.status === 'stopped' || result.status === 'success') {
            testRunning = false;
            updateStatus('status-test', 'Stopped', 'stopped');
            document.getElementById('btn-start-test').disabled = false;
            document.getElementById('btn-stop-test').disabled = true;
            stopElapsedTimer();
            log('Pattern test stopped');
        }
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

// ==================== Joint Control ====================

async function moveToHome() {
    try {
        log('Moving to home position...');
        const result = await api.moveToHome();
        log(`Move to home: ${result.status || 'success'}`);
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

async function moveToZero() {
    try {
        log('Moving to zero position...');
        const result = await api.moveToZero();
        log(`Move to zero: ${result.status || 'success'}`);
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

// ==================== Utilities ====================

function enableTestButtons(enabled) {
    document.getElementById('btn-preview').disabled = !enabled;
    document.getElementById('btn-start-test').disabled = !enabled;
    document.getElementById('btn-start-path').disabled = !enabled;
}

function updateStatus(elementId, text, state) {
    const el = document.getElementById(elementId);
    if (el) {
        el.textContent = text;
        el.className = `status-indicator status-${state}`;
    }
}

function updateProgress(percent, currentPoint, totalPoints, iteration, totalIterations) {
    document.getElementById('progress-fill').style.width = `${percent}%`;
    document.getElementById('progress-text').textContent = `${Math.round(percent)}%`;
    document.getElementById('current-point').textContent = `${currentPoint} / ${totalPoints}`;
    document.getElementById('current-iteration').textContent = `${iteration} / ${totalIterations}`;
}

function updatePosition(x, y, z) {
    document.getElementById('pos-x').textContent = x.toFixed(4);
    document.getElementById('pos-y').textContent = y.toFixed(4);
    document.getElementById('pos-z').textContent = z.toFixed(4);
}

function startElapsedTimer() {
    stopElapsedTimer();
    elapsedInterval = setInterval(() => {
        if (testStartTime) {
            const elapsed = Date.now() - testStartTime;
            const seconds = Math.floor(elapsed / 1000);
            const minutes = Math.floor(seconds / 60);
            const secs = seconds % 60;
            document.getElementById('elapsed-time').textContent =
                `${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
        }
    }, 1000);
}

function stopElapsedTimer() {
    if (elapsedInterval) {
        clearInterval(elapsedInterval);
        elapsedInterval = null;
    }
}

function log(message, level = 'info') {
    const logEl = document.getElementById('output-log');
    const timestamp = new Date().toLocaleTimeString();
    const prefix = level === 'error' ? '[ERR]' : '[INFO]';
    logEl.textContent += `[${timestamp}] ${prefix} ${message}\n`;
    logEl.scrollTop = logEl.scrollHeight;
}

function clearOutput() {
    document.getElementById('output-log').textContent = '';
}

function copyOutput() {
    const text = document.getElementById('output-log').textContent;
    navigator.clipboard.writeText(text).then(() => {
        log('Output copied to clipboard');
    });
}

function updateTimestamp() {
    const el = document.getElementById('last-update');
    if (el) el.textContent = new Date().toLocaleTimeString();
}
