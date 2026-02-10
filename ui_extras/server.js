const express = require('express');
const https = require('https');
const http = require('http');
const fs = require('fs');
const path = require('path');

const app = express();
const HTTPS_PORT = 3443;
const HTTP_PORT = 3080;
const HOST = process.env.CHAT_HOST;

// Enable CORS for all routes
app.use((req, res, next) => {
    res.header('Access-Control-Allow-Origin', '*');
    res.header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
    res.header('Access-Control-Allow-Headers', 'Content-Type');
    
    // Set correct MIME types
    if (req.path.endsWith('.js')) {
        res.type('application/javascript');
    } else if (req.path.endsWith('.css')) {
        res.type('text/css');
    }
    
    next();
});

// Serve static files from chat-widgets directory
app.use('/chat-widgets', express.static(path.join(__dirname, 'chat-widgets')));

// Serve root
app.use('/', express.static(path.join(__dirname)));

// Health check
app.get('/health', (req, res) => {
    res.json({ status: 'ok', message: 'Chat widgets server is running' });
});

// Log requests
app.use((req, res, next) => {
    console.log(`${new Date().toISOString()} - ${req.method} ${req.path}`);
    next();
});

// Try to load or create self-signed certificate
let options = {};

const certPath = path.join(__dirname, 'server.crt');
const keyPath = path.join(__dirname, 'server.key');

if (fs.existsSync(certPath) && fs.existsSync(keyPath)) {
    console.log('Using existing SSL certificate...');
    options.cert = fs.readFileSync(certPath);
    options.key = fs.readFileSync(keyPath);
    
    // Start HTTPS server
    https.createServer(options, app).listen(HTTPS_PORT, HOST, () => {
        console.log(`Chat widgets server running on https://${HOST}:${HTTPS_PORT}`);
        console.log(`Access files at: https://${HOST}:${HTTPS_PORT}/chat-widgets/`);
    });
} else {
    console.log('WARNING: Self-signed SSL certificates not found!');
    console.log('To create them, run:');
    console.log('  openssl req -x509 -newkey rsa:4096 -nodes -out server.crt -keyout server.key -days 365');
}

// Always start HTTP server as well for cross-origin compatibility
http.createServer(app).listen(HTTP_PORT, HOST, () => {
    console.log(`Chat widgets HTTP server running on http://${HOST}:${HTTP_PORT}`);
    console.log(`Access files at: http://${HOST}:${HTTP_PORT}/chat-widgets/`);
    console.log('');
    console.log('For phntm_bridge.yaml, use HTTP URLs (more reliable for cross-origin):');
    console.log('  ui_custom_includes_js:');
    console.log('    - http://192.168.2.14:3080/chat-widgets/chat-spot-widget.js');
    console.log('    - http://192.168.2.14:3080/chat-widgets/chat-drone-widget.js');
    console.log('    - http://192.168.2.14:3080/chat-widgets/chat-operator-widget.js');
    console.log('  ui_custom_includes_css:');
    console.log('    - http://192.168.2.14:3080/chat-widgets/chat-widgets.css');
});

process.on('SIGINT', () => {
    console.log('\nShutting down server...');
    process.exit(0);
});
