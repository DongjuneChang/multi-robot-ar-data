/**
 * Config Utilities
 * 공통 유틸: deepMerge, loadYaml
 * network_robots와 ros2_interface 양쪽에서 사용
 */

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

    // Use js-yaml if available
    if (typeof jsyaml !== 'undefined') {
        return jsyaml.load(text);
    } else if (typeof YAML !== 'undefined') {
        return YAML.parse(text);
    } else {
        console.warn('js-yaml not found, attempting JSON parse');
        return JSON.parse(text);
    }
}

/**
 * Load config with defaults + override merge pattern
 * @param {string} baseUrl - Base URL (e.g., '/config/ros2_interface')
 * @param {string} name - Config name (e.g., 'lite6')
 * @returns {Promise<Object>} Merged config
 */
async function loadMergedConfig(baseUrl, name) {
    // Load defaults
    const defaults = await loadYaml(`${baseUrl}/_defaults.yaml`);

    // Load override (may not exist)
    let override = {};
    try {
        override = await loadYaml(`${baseUrl}/overrides/${name}.yaml`);
    } catch (e) {
        console.warn(`No override found for ${name}, using defaults only`);
    }

    // Merge and return
    return deepMerge(defaults, override);
}

// Export for browser global
if (typeof window !== 'undefined') {
    window.ConfigUtils = {
        deepMerge,
        loadYaml,
        loadMergedConfig
    };
}

// Export for ES6 modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { deepMerge, loadYaml, loadMergedConfig };
}
