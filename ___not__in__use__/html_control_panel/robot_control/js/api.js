/**
 * FastAPI REST API Client
 * Handles all HTTP communication with the FastAPI backend
 */

class APIClient {
    constructor(baseUrl = null) {
        // Auto-detect server URL from current page location
        this.baseUrl = baseUrl || `${window.location.protocol}//${window.location.host}`;
        this.timeout = 10000; // 10 seconds default timeout
    }

    /**
     * Set the base URL for API calls
     */
    setBaseUrl(url) {
        this.baseUrl = url;
        console.log(`[API] Base URL set to: ${this.baseUrl}`);
    }

    /**
     * Generic fetch wrapper with error handling
     */
    async request(endpoint, options = {}) {
        const url = `${this.baseUrl}${endpoint}`;
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), options.timeout || this.timeout);

        try {
            const response = await fetch(url, {
                ...options,
                signal: controller.signal,
                headers: {
                    'Content-Type': 'application/json',
                    ...options.headers
                }
            });

            clearTimeout(timeoutId);

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            // Check if response is JSON
            const contentType = response.headers.get('content-type');
            if (contentType && contentType.includes('application/json')) {
                return await response.json();
            } else if (contentType && (contentType.includes('text/yaml') || contentType.includes('text/plain'))) {
                return await response.text();
            }

            return await response.text();
        } catch (error) {
            clearTimeout(timeoutId);
            if (error.name === 'AbortError') {
                throw new Error(`Request timeout: ${endpoint}`);
            }
            throw error;
        }
    }

    // ==================== Robot Configuration ====================

    /**
     * Get list of available robots
     */
    async getRobots() {
        return await this.request('/api/robots');
    }

    /**
     * Get robot configuration YAML
     */
    async getRobotConfig(robotType) {
        return await this.request(`/api/robots/${robotType}/robot.yaml`);
    }

    /**
     * Get robot frames YAML
     */
    async getRobotFrames(robotType) {
        return await this.request(`/api/robots/${robotType}/frames.yaml`);
    }

    // ==================== Config Files ====================

    /**
     * Get device configuration
     */
    async getDeviceConfig() {
        return await this.request('/api/config/device_config.yaml');
    }

    /**
     * Get network configuration
     */
    async getNetworkConfig() {
        return await this.request('/api/config/network_config.yaml');
    }

    /**
     * Get map data
     */
    async getMapData() {
        return await this.request('/api/config/map_data.yaml');
    }

    // ==================== System Control ====================

    /**
     * Clean system (kill all ROS2 processes)
     */
    async cleanSystem() {
        return await this.request('/api/system/clean', { method: 'POST' });
    }

    /**
     * Toggle MoveIt
     */
    async toggleMoveIt(action, mode = 'fake') {
        return await this.request(`/api/moveit/${action}`, {
            method: 'POST',
            body: JSON.stringify({ mode })
        });
    }

    /**
     * Toggle Controller
     */
    async toggleController(action) {
        return await this.request(`/api/controller/${action}`, { method: 'POST' });
    }

    /**
     * Toggle Visualizer
     */
    async toggleVisualizer(action) {
        return await this.request(`/api/visualizer/${action}`, { method: 'POST' });
    }

    /**
     * Toggle TCP Endpoint
     */
    async toggleTcpEndpoint(action) {
        return await this.request(`/api/tcp_endpoint/${action}`, { method: 'POST' });
    }

    /**
     * Toggle World TF
     */
    async toggleWorldTf(action) {
        return await this.request(`/api/world_tf/${action}`, { method: 'POST' });
    }

    /**
     * Toggle Visualization System
     */
    async toggleVizSystem(action) {
        return await this.request(`/api/viz_system/${action}`, { method: 'POST' });
    }

    /**
     * Toggle Foxglove Bridge
     */
    async toggleFoxglove(action) {
        return await this.request(`/api/foxglove/${action}`, { method: 'POST' });
    }

    // ==================== Path Planning ====================

    /**
     * Start path planning
     */
    async startPathPlanning() {
        return await this.request('/api/path/start', { method: 'POST' });
    }

    /**
     * Stop path planning
     */
    async stopPathPlanning() {
        return await this.request('/api/path/stop', { method: 'POST' });
    }

    // ==================== Pattern Test ====================

    /**
     * Toggle pattern service
     */
    async togglePatternService(action, params = {}) {
        return await this.request(`/api/pattern/service/${action}`, {
            method: 'POST',
            body: JSON.stringify(params)
        });
    }

    /**
     * Preview pattern test
     */
    async previewPatternTest() {
        return await this.request('/api/pattern/preview', { method: 'POST' });
    }

    /**
     * Start pattern test
     */
    async startPatternTest() {
        return await this.request('/api/pattern/start', { method: 'POST' });
    }

    /**
     * Stop pattern test
     */
    async stopPatternTest() {
        return await this.request('/api/pattern/stop', { method: 'POST' });
    }

    // ==================== Device Management ====================

    /**
     * Toggle Device API
     */
    async toggleDeviceApi(action) {
        return await this.request(`/api/device_api/${action}`, { method: 'POST' });
    }

    /**
     * Scan for AR devices
     */
    async scanDevices() {
        return await this.request('/api/devices/scan', { method: 'POST', timeout: 15000 });
    }

    /**
     * Get discovered devices
     */
    async getDevices() {
        return await this.request('/api/devices');
    }

    // ==================== Status ====================

    /**
     * Get full system status
     */
    async getStatus() {
        return await this.request('/api/status');
    }

    /**
     * Get Foxglove Bridge status
     */
    async getFoxgloveStatus() {
        return await this.request('/api/foxglove/status');
    }

    // ==================== Joint Control ====================

    /**
     * Set joint angles
     */
    async setJoints(joints) {
        return await this.request('/api/joints/set', {
            method: 'POST',
            body: JSON.stringify({ joints })
        });
    }

    /**
     * Move to home position
     */
    async moveToHome() {
        return await this.request('/api/joints/home', { method: 'POST' });
    }

    /**
     * Move to zero position
     */
    async moveToZero() {
        return await this.request('/api/joints/zero', { method: 'POST' });
    }
}

// Global API client instance
const api = new APIClient();
