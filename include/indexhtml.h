#ifndef INDEX_HTML
#define INDEX_HTML

#include <pgmspace.h>

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Xroute Tester - Styled Controls</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            background-color: #1e1e2f;
            color: #e0e0ff;
            display: flex;
            justify-content: center;
            align-items: flex-start;
            min-height: 100vh;
            padding: 20px;
            box-sizing: border-box;
        }

        .container {
            background-color: #2a2a45;
            padding: 20px;
            border-radius: 12px;
            box-shadow: 0 8px 25px rgba(0,0,0,0.3);
            width: 100%;
            max-width: 700px;
        }

        h2 {
            color: #c0c0ff;
            text-align: center;
            margin-bottom: 25px;
            font-weight: 600;
        }

        h3 {
            color: #a0a0cf;
            margin-top: 30px;
            margin-bottom: 15px;
            border-bottom: 1px solid #404066;
            padding-bottom: 8px;
            font-weight: 500;
        }

        .dimmers-list {
            display: flex;
            flex-direction: column;
            gap: 15px;
            margin-bottom: 20px;
        }

        .dimmer {
            display: flex;
            align-items: center;
            padding: 12px 15px;
            background-color: #363655;
            border-radius: 10px;
            gap: 15px;
        }

        .dimmer-icon {
            font-size: 22px;
            color: #8080ff;
        }

        .dimmer-label {
            font-size: 0.95em;
            color: #c0c0ff;
            min-width: 30px;
        }

        input[type="range"].dimmer-slider {
            flex-grow: 1;
            height: 8px;
            background: #2a2a45;
            border-radius: 5px;
            -webkit-appearance: none;
            appearance: none;
            cursor: pointer;
        }
        input[type="range"].dimmer-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            background: #8080ff;
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid #363655;
        }
        input[type="range"].dimmer-slider::-moz-range-thumb {
            width: 18px;
            height: 18px;
            background: #8080ff;
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid #363655;
        }

        .dimmer-value-display {
            font-size: 1em;
            font-weight: bold;
            color: #e0e0ff;
            min-width: 35px;
            text-align: right;
        }
        
        .dimmer.disabled input[type="range"].dimmer-slider::-webkit-slider-thumb {
            background: #555577;
        }
        .dimmer.disabled input[type="range"].dimmer-slider::-moz-range-thumb {
            background: #555577;
        }
        .dimmer.disabled {
            cursor: not-allowed;
            opacity: 0.6;
        }
        .dimmer.disabled input[type="range"].dimmer-slider {
             cursor: not-allowed;
        }

        .controls-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(100px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        .switch-button {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            padding: 15px;
            background-color: #363655;
            border-radius: 10px;
            cursor: pointer;
            text-align: center;
            min-height: 100px;
            transition: background-color 0.3s ease, box-shadow 0.3s ease;
            user-select: none;
        }
        .switch-button.on {
            background-color: #5050a0;
            box-shadow: 0 0 15px rgba(80, 80, 160, 0.7);
        }
        .switch-icon {
            font-size: 28px;
            color: #c0c0ff;
            margin-bottom: 10px;
        }
        .switch-button.on .switch-icon { color: #e0e0ff; }
        .switch-label {
            font-size: 0.9em;
            color: #c0c0ff;
        }
        .switch-button.on .switch-label { color: #e0e0ff; font-weight: 500; }
        .switch-button.disabled {
            background-color: #404055 !important;
            cursor: not-allowed;
            opacity: 0.6;
        }

        pre#log {
            background: #2a2a45;
            padding: 1em;
            border: 1px solid #404066;
            border-radius: 8px;
            white-space: pre-wrap;
            word-wrap: break-word;
            max-height: 200px;
            overflow-y: auto;
            margin-top: 20px;
            font-size: 0.85em;
            color: #b0b0df;
        }
    </style>
</head>
<body>
  <div class="container">
    <h2>Xroute Test - Controls</h2>

    <h3>Dimmers</h3>
    <div id="dimmersContainer" class="dimmers-list">
      </div>

    <h3>Switches</h3>
    <div id="switchesContainer" class="controls-grid">
      </div>

    <pre id="log"></pre>
  </div>

  <script>
    const logElement = document.getElementById('log');
    let ws;
    const websocketUrl = "ws://xroute.local:8080/"; // Make sure this is the correct address for your Xroute device

    const numberOfDimmers = 5;
    const dimmerIcons = ['☀️', '💡', '🔆', '🌟', '✨'];
    const dimmerData = {}; // Stores { updateVisual: func, lastSentQuantized: num }

    const numberOfSwitches = 8;
    const switchUiData = [ // <<<< THIS IS THE MODIFIED SECTION >>>>
        { icon: '🔌', label: 'USB' },      // Corresponds to sw1
        { icon: '🔥', label: 'Boiler' },   // Corresponds to sw2
        { icon: '🧊', label: 'Fridge' },   // Corresponds to sw3 (placeholder icon)
        { icon: '🚽', label: 'WC' },       // Corresponds to sw4
        { icon: '💡', label: 'Lamp' },     // Corresponds to sw5 (placeholder icon)
        { icon: '💧', label: 'Pump' },     // Corresponds to sw6
        { icon: '💧', label: 'Valve' },  // Corresponds to sw7 (placeholder icon)
        { icon: '램프', label: 'Reading Lamp' } // Corresponds to sw8 (placeholder icon)
    ];
    let initialStatusReceived = false;

    const log = msg => {
      const now = new Date();
      const timeString = now.toLocaleTimeString();
      const textNode = document.createTextNode(`[${timeString}] ${msg}\n`);
      logElement.appendChild(textNode);
      logElement.scrollTop = logElement.scrollHeight;
    };

    function setAllControlsDisabled(disabled) {
        document.querySelectorAll('.dimmer, .switch-button').forEach(control => {
            if (disabled) {
                control.classList.add('disabled');
            } else {
                control.classList.remove('disabled');
            }
            const sliderInput = control.querySelector('input[type="range"]');
            if (sliderInput) {
                sliderInput.disabled = disabled;
            }
        });
    }

    function connectWebSocket() {
      log(`Attempting to connect to WebSocket at ${websocketUrl}...`);
      initialStatusReceived = false;
      ws = new WebSocket(websocketUrl);

      ws.onopen = () => {
        log("WS: Connection opened. Waiting for initial status...");
        // Request initial status explicitly if your device supports/requires it
        // Example: if (ws.readyState === WebSocket.OPEN) ws.send("GET_STATUS\n");
      };

      ws.onmessage = ev => {
        log("WS recv: " + ev.data);
        try {
            const receivedData = JSON.parse(ev.data);
            // Refined check for initial status: look for specific keys that indicate a full status update
            if (!initialStatusReceived &&
                receivedData.hasOwnProperty('voltage') && // Example key, adjust to your device's status message
                (receivedData.hasOwnProperty('DIMER1') || receivedData.hasOwnProperty('sw1'))) { // Ensure at least one control key is present
                log("Received initial status message via WebSocket.");
                initializeControls(receivedData);
                initialStatusReceived = true;
                setAllControlsDisabled(false);
                log("Initial status processed, controls enabled.");
            } else if (initialStatusReceived) {
                // Handle incremental updates if your device sends them
                updateControls(receivedData);
            }
        } catch (e) {
            // Log non-JSON messages if they are expected for other purposes
            // console.warn("Received non-JSON message or not an init message:", ev.data, e);
        }
      };

      ws.onclose = () => {
        log("WS: Connection closed. Reconnecting in 3s...");
        setAllControlsDisabled(true);
        initialStatusReceived = false;
        setTimeout(connectWebSocket, 3000);
      };

      ws.onerror = (error) => {
        log("WS error: " + (error.message || "Unknown error. Check console for details."));
        console.error("WS error object:", error);
        setAllControlsDisabled(true);
        initialStatusReceived = false;
        // Optionally, you might not want to immediately reconnect on all errors,
        // or implement a backoff strategy for retries.
      };
    }

    function initializeControls(statusData) {
        if (typeof statusData !== 'object' || statusData === null) {
            log("Error: initializeControls received invalid statusData.");
            console.error("Invalid statusData:", statusData);
            return;
        }

        // Initialize Dimmers
        for (let i = 1; i <= numberOfDimmers; i++) {
            const dimmerKey = `DIMER${i}`;
            if (statusData.hasOwnProperty(dimmerKey) && dimmerData[i] && typeof dimmerData[i].updateVisual === 'function') {
                const value = parseInt(statusData[dimmerKey], 10);
                if (!isNaN(value)) {
                    dimmerData[i].updateVisual(value);
                    dimmerData[i].lastSentQuantized = Math.min(255, Math.round(value / 8) * 8);
                } else {
                    log(`Warning: Invalid value for ${dimmerKey} in status: ${statusData[dimmerKey]}.`);
                }
            }
        }

        // Initialize Switches
        const switchElements = document.querySelectorAll('.switch-button');
        switchElements.forEach(switchEl => {
            const switchNumber = switchEl.dataset.switchNumber;
            const switchKey = `sw${switchNumber}`;
            if (statusData.hasOwnProperty(switchKey)) {
                const stateFromServer = String(statusData[switchKey]).toUpperCase(); // Normalize state
                const isOn = stateFromServer === "ON";
                switchEl.dataset.isOn = isOn ? 'true' : 'false';
                if (isOn) {
                    switchEl.classList.add('on');
                } else {
                    switchEl.classList.remove('on');
                }
            }
        });
    }

    // Optional: Function to handle incremental updates after initial load
    function updateControls(updateData) {
        if (typeof updateData !== 'object' || updateData === null) return;

        // Update Dimmers
        for (let i = 1; i <= numberOfDimmers; i++) {
            const dimmerKey = `DIMER${i}`;
            if (updateData.hasOwnProperty(dimmerKey) && dimmerData[i] && typeof dimmerData[i].updateVisual === 'function') {
                const value = parseInt(updateData[dimmerKey], 10);
                if (!isNaN(value)) {
                    dimmerData[i].updateVisual(value);
                    // Update lastSentQuantized if needed, though typically this is for user input
                    // dimmerData[i].lastSentQuantized = Math.min(255, Math.round(value / 8) * 8);
                }
            }
        }

        // Update Switches
        const switchElements = document.querySelectorAll('.switch-button');
        switchElements.forEach(switchEl => {
            const switchNumber = switchEl.dataset.switchNumber;
            const switchKey = `sw${switchNumber}`;
            if (updateData.hasOwnProperty(switchKey)) {
                const stateFromServer = String(updateData[switchKey]).toUpperCase(); // Normalize state
                const isOn = stateFromServer === "ON";
                const currentIsOn = switchEl.dataset.isOn === 'true';
                if (isOn !== currentIsOn) { // Only update if state changed
                    switchEl.dataset.isOn = isOn ? 'true' : 'false';
                    if (isOn) {
                        switchEl.classList.add('on');
                    } else {
                        switchEl.classList.remove('on');
                    }
                    log(`Switch ${switchNumber} updated to ${stateFromServer} from server.`);
                }
            }
        });
    }


    // Create Dimmers UI
    const dimmersContainer = document.getElementById('dimmersContainer');
    for (let i = 1; i <= numberOfDimmers; i++) {
        const dimmerEl = document.createElement('div');
        dimmerEl.className = 'dimmer';
        dimmerEl.dataset.dimmerNumber = i;

        const iconEl = document.createElement('div');
        iconEl.className = 'dimmer-icon';
        iconEl.textContent = dimmerIcons[i-1] || '🔆';

        const labelEl = document.createElement('label');
        labelEl.className = 'dimmer-label';
        labelEl.textContent = `D${i}:`;
        labelEl.htmlFor = `dimmerSlider${i}`;

        const sliderEl = document.createElement('input');
        sliderEl.type = 'range';
        sliderEl.id = `dimmerSlider${i}`;
        sliderEl.className = 'dimmer-slider';
        sliderEl.min = '0';
        sliderEl.max = '255';

        const valueDisplayEl = document.createElement('span');
        valueDisplayEl.className = 'dimmer-value-display';

        dimmerEl.appendChild(iconEl);
        dimmerEl.appendChild(labelEl);
        dimmerEl.appendChild(sliderEl);
        dimmerEl.appendChild(valueDisplayEl);
        dimmersContainer.appendChild(dimmerEl);

        const updateVisual = (value) => {
            const clampedValue = Math.max(0, Math.min(255, parseInt(value, 10) || 0));
            sliderEl.value = clampedValue;
            valueDisplayEl.textContent = clampedValue;
        };
        
        dimmerData[i] = {
            updateVisual: updateVisual,
            lastSentQuantized: -1 // Initialize to a value that won't match
        };
        dimmerData[i].updateVisual(0); // Set initial visual state

        let debounceTimer;
        sliderEl.oninput = () => { // While dragging
            if (dimmerEl.classList.contains('disabled')) return;
            
            const currentValue = parseInt(sliderEl.value, 10);
            dimmerData[i].updateVisual(currentValue); // Update UI immediately

            clearTimeout(debounceTimer);
            debounceTimer = setTimeout(() => {
                let quantizedValue = Math.round(currentValue / 8) * 8;
                quantizedValue = Math.min(255, quantizedValue); // Ensure it doesn't exceed 255

                if (quantizedValue !== dimmerData[i].lastSentQuantized) {
                    if (ws && ws.readyState === WebSocket.OPEN) {
                        const command = `DIMER${i}=${quantizedValue}\n`;
                        ws.send(command);
                        log(`WS send (drag-quantized): ${command.replace(/\n$/, '\\n')}`);
                        dimmerData[i].lastSentQuantized = quantizedValue;
                    }
                }
            }, 150); // Adjust debounce delay as needed (e.g., 150ms)
        };

        sliderEl.onchange = () => { // On release
            if (dimmerEl.classList.contains('disabled')) return;
            clearTimeout(debounceTimer); // Clear any pending debounced send

            const finalValue = parseInt(sliderEl.value, 10);
            dimmerData[i].updateVisual(finalValue);

            if (ws && ws.readyState === WebSocket.OPEN) {
                const command = `DIMER${i}=${finalValue}\n`; // Send the precise final value
                ws.send(command);
                log(`WS send (release): ${command.replace(/\n$/, '\\n')}`);
            }
            // Update lastSentQuantized based on the final value, ensuring it's also clamped
            dimmerData[i].lastSentQuantized = Math.min(255, Math.round(finalValue / 8) * 8);
        };
    }

    // Create Switches UI
    const switchesContainer = document.getElementById('switchesContainer');
    for (let i = 1; i <= numberOfSwitches; i++) {
        const switchEl = document.createElement('div');
        switchEl.className = 'switch-button';
        switchEl.dataset.switchNumber = i;

        const iconEl = document.createElement('div');
        iconEl.className = 'switch-icon';
        iconEl.textContent = switchUiData[i-1] ? switchUiData[i-1].icon : '❓';

        const labelEl = document.createElement('div');
        labelEl.className = 'switch-label';
        labelEl.textContent = switchUiData[i-1] ? switchUiData[i-1].label : `Switch ${i}`;

        switchEl.appendChild(iconEl);
        switchEl.appendChild(labelEl);
        switchesContainer.appendChild(switchEl);
        
        // Initial state is false, will be updated by initializeControls from server data
        switchEl.dataset.isOn = 'false';

        switchEl.addEventListener('click', () => {
            if (switchEl.classList.contains('disabled')) return;
            const currentlyOn = switchEl.dataset.isOn === 'true';
            const newStateOn = !currentlyOn;
            // Update UI immediately for responsiveness
            switchEl.dataset.isOn = newStateOn ? 'true' : 'false';
            if (newStateOn) {
                switchEl.classList.add('on');
            } else {
                switchEl.classList.remove('on');
            }
            const stateStr = newStateOn ? 'ON' : 'OFF';
            if (ws && ws.readyState === WebSocket.OPEN) {
                const command = `sw${i}=${stateStr}\n`;
                ws.send(command);
                log(`WS send: ${command.replace(/\n$/, '\\n')}`);
            } else {
                log(`WS not open. Switch ${i} command not sent.`);
                // Optionally revert UI change if send fails and matters
                // switchEl.dataset.isOn = currentlyOn ? 'true' : 'false';
                // if (currentlyOn) { switchEl.classList.add('on'); } else { switchEl.classList.remove('on'); }
            }
        });
    }

    setAllControlsDisabled(true); // Disable controls until WebSocket connection and initial status
    connectWebSocket();
  </script>
</body>
</html>
)rawliteral";

#endif // INDEX_HTML
