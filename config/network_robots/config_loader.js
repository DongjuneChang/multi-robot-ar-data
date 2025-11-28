/**
 * Network Robot Config Loader (JavaScript)
 * 병합 로더: _defaults.yaml + overrides/{robot}.yaml → 완전한 config
 *
 * Usage:
 *   import { loadRobotConfig } from './config_loader.js';
 *   const config = await loadRobotConfig('lite6');
 */

// js-yaml 필요: npm install js-yaml
// 또는 브라우저에서 CDN: <script src="https://cdn.jsdelivr.net/npm/js-yaml@4.1.0/dist/js-yaml.min.js"></script>

/**
 * Deep merge override into base
 * @param {Object} base - Base object
 * @param {Object} override - Override object
 * @returns {Object} Merged object
 */
function deepMerge(base, override) {
    const result = JSON.parse(JSON.stringify(base)); // Deep copy

    for (const key in override) {
        if (override.hasOwnProperty(key)) {
            if (result[key] && typeof result[key] === 'object' && typeof override[key] === 'object'
                && !Array.isArray(result[key]) && !Array.isArray(override[key])) {
                result[key] = deepMerge(result[key], override[key]);
            } else {
                result[key] = JSON.parse(JSON.stringify(override[key]));
            }
        }
    }

    return result;
}

/**
 * Load YAML file
 * @param {string} url - URL to YAML file
 * @returns {Promise<Object>} Parsed YAML content
 */
async function loadYaml(url) {
    const response = await fetch(url);
    if (!response.ok) {
        throw new Error(`Failed to load: ${url}`);
    }
    const text = await response.text();

    // Use js-yaml if available, otherwise simple parse
    if (typeof jsyaml !== 'undefined') {
        return jsyaml.load(text);
    } else if (typeof YAML !== 'undefined') {
        return YAML.parse(text);
    } else {
        // Fallback: assume JSON-compatible YAML
        console.warn('js-yaml not found, attempting JSON parse');
        return JSON.parse(text);
    }
}

/**
 * Get base URL for config files
 * @returns {string} Base URL
 */
function getConfigBaseUrl() {
    // Adjust based on your server setup
    return '/config/network_robots';
}

/**
 * Load robot configuration (defaults + override merged)
 * @param {string} robotModel - Robot model name (e.g., "lite6", "xarm5")
 * @param {Object} options - Options
 * @param {boolean} options.convertToRadians - Convert joint angles to radians
 * @param {string} options.baseUrl - Custom base URL
 * @returns {Promise<Object>} Merged config
 */
async function loadRobotConfig(robotModel, options = {}) {
    const { convertToRadians = false, baseUrl = getConfigBaseUrl() } = options;

    // Load defaults
    const defaults = await loadYaml(`${baseUrl}/_defaults.yaml`);

    // Load override
    const override = await loadYaml(`${baseUrl}/overrides/${robotModel}.yaml`);

    // Merge
    let config = deepMerge(defaults, override);

    // Convert to radians if requested
    if (convertToRadians) {
        config = convertJointsToRadians(config);
    }

    return config;
}

/**
 * Convert joint angles from degrees to radians
 * @param {Object} config - Config object
 * @returns {Object} Config with radians
 */
function convertJointsToRadians(config) {
    const result = JSON.parse(JSON.stringify(config));
    const deg2rad = (deg) => deg * Math.PI / 180;

    if (result.joints) {
        if (result.joints.zero) {
            result.joints.zero = result.joints.zero.map(deg2rad);
        }
        if (result.joints.home) {
            result.joints.home = result.joints.home.map(deg2rad);
        }
        if (result.joints.limits) {
            for (const jointName in result.joints.limits) {
                result.joints.limits[jointName] = result.joints.limits[jointName].map(deg2rad);
            }
        }
    }

    return result;
}

/**
 * Load named_frames section only (for FrameManager)
 * @param {string} robotModel - Robot model name
 * @returns {Promise<Object>} named_frames config
 */
async function loadNamedFrames(robotModel) {
    const config = await loadRobotConfig(robotModel, { convertToRadians: true });
    return config.named_frames || {};
}

/**
 * Load admittance section only
 * @param {string} robotModel - Robot model name
 * @returns {Promise<Object>} admittance config
 */
async function loadAdmittanceConfig(robotModel) {
    const config = await loadRobotConfig(robotModel);
    return config.admittance || {};
}

/**
 * Get list of available robots
 * @param {string} baseUrl - Base URL
 * @returns {Promise<string[]>} List of robot model names
 */
async function getAvailableRobots(baseUrl = getConfigBaseUrl()) {
    // This requires server-side support or a manifest file
    // For now, return hardcoded list
    return [
        'lite6', 'xarm5', 'fr3',
        'ur3e', 'ur5e', 'ur16e',
        'iiwa7', 'med7', 'med14',
        'kinova_gen3_7dof'
    ];
}

// Export for ES6 modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = {
        loadRobotConfig,
        loadNamedFrames,
        loadAdmittanceConfig,
        getAvailableRobots,
        deepMerge,
        convertJointsToRadians
    };
}

// Export for browser global
if (typeof window !== 'undefined') {
    window.ConfigLoader = {
        loadRobotConfig,
        loadNamedFrames,
        loadAdmittanceConfig,
        getAvailableRobots,
        deepMerge,
        convertJointsToRadians
    };
}
