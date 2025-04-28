function setOrigin() {
    console.log("Set Origin command sent");
    document.getElementById("phase").innerText = "Origin Set";
    addToLog("Command: setOrigin()");
}

function startExploration() {
    console.log("Exploration started");
    document.getElementById("phase").innerText = "Mapping...";
    addToLog("Command: explore()");
}

function goToTarget() {
    const x = document.getElementById("xCoord").value;
    const y = document.getElementById("yCoord").value;
    console.log(`Navigating to (${x}, ${y})`);
    document.getElementById("phase").innerText = `Navigating to (${x}, ${y})`;
    addToLog(`Command: go(${x}, ${y})`);
}

function showTab(tab) {
    const buttons = document.querySelectorAll('.tab-button');
    const contents = document.querySelectorAll('.tab-content');

    buttons.forEach(btn => btn.classList.remove('active'));
    contents.forEach(c => c.classList.remove('active-tab'));

    if (tab === 'user') {
        document.getElementById('userView').classList.add('active-tab');
        buttons[0].classList.add('active');
    } else if (tab === 'debug') {
        document.getElementById('debugView').classList.add('active-tab');
        buttons[1].classList.add('active');
    }
}

function addToLog(text) {
    const log = document.getElementById("debugLog");
    log.innerText += "\n" + text;
}
