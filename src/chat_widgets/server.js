const express = require('express');
const https = require('https');
const http = require('http');
const fs = require('fs');
const path = require('path');

const app = express();
const HTTPS_PORT = process.env.CHAT_HTTPS_PORT || 3443;
const HTTP_PORT = process.env.CHAT_HTTP_PORT || 3080;
const HOST = process.env.CHAT_HOST || '0.0.0.0';

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

// Serve static files from current directory (chat widgets)
app.use('/chat-widgets', express.static(path.join(__dirname)));

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
const { execSync } = require('child_process');
let options = {};

const certPath = path.join(__dirname, 'server.crt');
const keyPath = path.join(__dirname, 'server.key');

function ensureCertificate() {
    if (fs.existsSync(certPath) && fs.existsSync(keyPath)) {
        console.log('Using existing SSL certificate...');
        return true;
    }
    
    try {
        console.log('Generating self-signed SSL certificate...');
        execSync(
            `openssl req -x509 -newkey rsa:2048 -keyout ${keyPath} -out ${certPath} -days 365 -nodes -subj "/CN=localhost"`,
            { stdio: 'pipe' }
        );
        console.log('✓ SSL certificate generated successfully');
        return true;
    } catch (error) {
        console.error('✗ Failed to generate SSL certificate:', error.message);
        return false;
    }
}

if (ensureCertificate()) {
    try {
        options.cert = fs.readFileSync(certPath);
        options.key = fs.readFileSync(keyPath);
        
        // Start HTTPS server
        https.createServer(options, app).listen(HTTPS_PORT, HOST, () => {
            console.log(`Chat widgets HTTPS server running on https://${HOST}:${HTTPS_PORT}`);
            console.log(`Access files at: https://${HOST}:${HTTPS_PORT}/chat-widgets/`);
        });
    } catch (error) {
        console.error('✗ Failed to start HTTPS server:', error.message);
    }
} else {
    console.error('✗ Could not create SSL certificates. HTTPS server will not start.');
}

// Always start HTTP server as well for fallback
http.createServer(app).listen(HTTP_PORT, HOST, () => {
    console.log(`Chat widgets HTTP server running on http://${HOST}:${HTTP_PORT}`);
    console.log(`Access files at: http://${HOST}:${HTTP_PORT}/chat-widgets/`);
    console.log('');
    console.log('For phntm_bridge.yaml, use HTTPS URLs:');
    console.log('  ui_custom_includes_js:');
    console.log(`    - //localhost:${HTTPS_PORT}/chat-widgets/chat-spot-widget.js`);
    console.log(`    - //localhost:${HTTPS_PORT}/chat-widgets/chat-drone-widget.js`);
    console.log(`    - //localhost:${HTTPS_PORT}/chat-widgets/chat-operator-widget.js`);
    console.log('  ui_custom_includes_css:');
    console.log(`    - //localhost:${HTTPS_PORT}/chat-widgets/chat-widgets.css`);
});

process.on('SIGINT', () => {
    console.log('\nShutting down server...');
    process.exit(0);
});
