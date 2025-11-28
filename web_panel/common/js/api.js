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
     * Get robot configuration (merged defaults + override)
     */
    async getRobotConfig(robotType) {
        return await this.request(`/api/robots/${robotType}/config`);
    }

    /**
     * Get robot override YAML
     */
    async getRobotOverride(robotType) {
        return await this.request(`/api/robots/${robotType}/override`);
    }

    /**
     * Get robot defaults YAML
     */
    async getRobotDefaults(robotType) {
        return await this.request(`/api/robots/${robotType}/defaults`);
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
        return await this.request('/api/ros2/system/clean', { method: 'POST' });
    }

    /**
     * Toggle MoveIt
     */
    async toggleMoveIt(action, mode = 'fake') {
        return await this.request(`/api/ros2/moveit/${action}`, {
            method: 'POST',
            body: JSON.stringify({ mode })
        });
    }

    /**
     * Toggle Controller
     */
    async toggleController(action) {
        return await this.request(`/api/ros2/controller/${action}`, { method: 'POST' });
    }

    /**
     * Toggle Visualizer
     */
    async toggleVisualizer(action) {
        return await this.request(`/api/ros2/visualizer/${action}`, { method: 'POST' });
    }

    /**
     * Toggle TCP Endpoint
     * @param {string} action - 'start' or 'stop'
     * @param {string} server - Optional server ID from network_config
     */
    async toggleTcpEndpoint(action, server = null) {
        const options = { method: 'POST' };
        if (server) {
            options.body = JSON.stringify({ server });
        }
        return await this.request(`/api/ros2/tcp_endpoint/${action}`, options);
    }

    /**
     * Toggle World TF
     */
    async toggleWorldTf(action, server = null) {
        const options = { method: 'POST' };
        if (server) {
            options.body = JSON.stringify({ server });
        }
        return await this.request(`/api/ros2/world_tf/${action}`, options);
    }

    /**
     * Toggle Visualization System
     */
    async toggleVizSystem(action, server = null) {
        const options = { method: 'POST' };
        if (server) {
            options.body = JSON.stringify({ server });
        }
        return await this.request(`/api/ros2/viz_system/${action}`, options);
    }

    /**
     * Toggle Foxglove Bridge
     */
    async toggleFoxglove(action, server = null) {
        const options = { method: 'POST' };
        if (server) {
            options.body = JSON.stringify({ server });
        }
        return await this.request(`/api/ros2/foxglove/${action}`, options);
    }

    // ==================== Path Planning ====================

    /**
     * Start path planning
     */
    async startPathPlanning() {
        return await this.request('/api/ros2/path/start', { method: 'POST' });
    }

    /**
     * Stop path planning
     */
    async stopPathPlanning() {
        return await this.request('/api/ros2/path/stop', { method: 'POST' });
    }

    // ==================== Pattern Test ====================

    /**
     * Toggle pattern service
     */
    async togglePatternService(action, params = {}) {
        return await this.request(`/api/ros2/pattern/service/${action}`, {
            method: 'POST',
            body: JSON.stringify(params)
        });
    }

    /**
     * Preview pattern test
     */
    async previewPatternTest() {
        return await this.request('/api/ros2/pattern/preview', { method: 'POST' });
    }

    /**
     * Start pattern test
     */
    async startPatternTest() {
        return await this.request('/api/ros2/pattern/start', { method: 'POST' });
    }

    /**
     * Stop pattern test
     */
    async stopPatternTest() {
        return await this.request('/api/ros2/pattern/stop', { method: 'POST' });
    }

    // ==================== Device Management ====================

    /**
     * Toggle Device API
     * @param {string} action - 'start' or 'stop'
     * @param {string} server - Optional server ID from network_config
     */
    async toggleDeviceApi(action, server = null) {
        const options = { method: 'POST' };
        if (server) {
            options.body = JSON.stringify({ server });
        }
        return await this.request(`/api/ros2/device_api/${action}`, options);
    }

    /**
     * Scan for AR devices
     * @param {string} server - Optional server ID from network_config
     */
    async scanDevices(server = null) {
        const options = { method: 'POST', timeout: 15000 };
        if (server) {
            options.body = JSON.stringify({ server });
        }
        return await this.request('/api/ros2/devices/scan', options);
    }

    /**
     * Get discovered devices
     */
    async getDevices() {
        return await this.request('/api/ros2/devices');
    }

    // ==================== Status ====================

    /**
     * Get full system status
     */
    async getStatus() {
        return await this.request('/api/ros2/status');
    }

    /**
     * Get Foxglove Bridge status
     */
    async getFoxgloveStatus() {
        return await this.request('/api/ros2/foxglove/status');
    }

    // ==================== Joint Control ====================

    /**
     * Set joint angles
     */
    async setJoints(joints) {
        return await this.request('/api/ros2/joints/set', {
            method: 'POST',
            body: JSON.stringify({ joints })
        });
    }

    /**
     * Move to home position
     */
    async moveToHome() {
        return await this.request('/api/ros2/joints/home', { method: 'POST' });
    }

    /**
     * Move to zero position
     */
    async moveToZero() {
        return await this.request('/api/ros2/joints/zero', { method: 'POST' });
    }
}

// Global API client instance
const api = new APIClient();
