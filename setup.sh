#!/bin/bash

# Phantom Bridge Client - First Time Setup Script
# This script automates the initial setup for new users

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Functions
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

check_command() {
    if ! command -v $1 &> /dev/null; then
        return 1
    fi
    return 0
}

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Start setup
print_header "Phantom Bridge Client - Setup Wizard"

# Check prerequisites
print_info "Checking prerequisites..."

if ! check_command docker; then
    print_error "Docker is not installed"
    echo "Please install Docker from: https://docs.docker.com/engine/install/"
    exit 1
fi
print_success "Docker is installed"

# Check for Docker Compose (plugin first, then standalone)
DOCKER_COMPOSE=""
if docker compose version &> /dev/null; then
    DOCKER_COMPOSE="docker compose"
    print_success "Docker Compose (plugin) is installed"
elif check_command docker-compose; then
    DOCKER_COMPOSE="docker-compose"
    print_success "Docker Compose (standalone) is installed"
else
    print_error "Docker Compose is not installed"
    echo "Please install Docker Compose: https://docs.docker.com/compose/install/"
    exit 1
fi

# Check for wget or curl (needed for robot registration)
if check_command wget; then
    DOWNLOADER="wget"
    print_success "wget is available"
elif check_command curl; then
    DOWNLOADER="curl"
    print_success "curl is available"
else
    print_error "Neither wget nor curl is installed"
    echo "Please install wget or curl:"
    echo "  sudo apt install wget"
    exit 1
fi

# Check if user can run Docker
if ! docker ps &> /dev/null; then
    print_error "Cannot run Docker commands. You may need to add your user to the docker group:"
    echo "  sudo usermod -aG docker \${USER}"
    echo "  # Log out and back in"
    exit 1
fi
print_success "Docker is accessible"

print_header "Step 1: Robot Registration"

# Check if config file exists
if [ -f "$HOME/phntm_bridge.yaml" ]; then
    print_warning "Config file already exists at ~/phntm_bridge.yaml"
    read -p "Do you want to register a new robot? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Downloading new config from Bridge Server..."
        if [ "$DOWNLOADER" = "wget" ]; then
            wget -O "$HOME/phntm_bridge.yaml" 'https://register.phntm.io/robot?yaml'
        else
            curl -fSL -o "$HOME/phntm_bridge.yaml" 'https://register.phntm.io/robot?yaml'
        fi
        print_success "New robot registered and config saved to ~/phntm_bridge.yaml"
    else
        print_info "Using existing config at ~/phntm_bridge.yaml"
    fi
else
    print_info "No config file found. Registering new robot with Bridge Server..."
    if [ "$DOWNLOADER" = "wget" ]; then
        wget -O "$HOME/phntm_bridge.yaml" 'https://register.phntm.io/robot?yaml'
    else
        curl -fSL -o "$HOME/phntm_bridge.yaml" 'https://register.phntm.io/robot?yaml'
    fi
    print_success "Robot registered and config saved to ~/phntm_bridge.yaml"
fi

# Extract robot ID from config
ROBOT_ID=$(grep "id_robot:" "$HOME/phntm_bridge.yaml" | head -1 | sed 's/.*: //' | tr -d ' ')

if [ -z "$ROBOT_ID" ]; then
    print_error "Could not extract robot ID from config file"
    exit 1
fi

print_success "Robot ID: $ROBOT_ID"

print_header "Step 2: Architecture Detection"

ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    print_info "Detected x86_64 architecture."
    export DOCKERFILE="Dockerfile.x64"
    export IMAGE_TAG="humble-x86"
elif [ "$ARCH" = "aarch64" ]; then
    print_info "Detected ARM64 architecture (likely Jetson)."
    export DOCKERFILE="Dockerfile"
    export IMAGE_TAG="humble-arm"
else
    print_warning "Unknown architecture: $ARCH. Defaulting to 'x86' configuration."
    export DOCKERFILE="Dockerfile.x64"
    export IMAGE_TAG="humble-x86"
fi

print_header "Step 3: Building Docker Image"

print_info "Building Docker image using $DOCKERFILE (tag: $IMAGE_TAG)..."
# Cleanup old containers first to avoid name conflicts
$DOCKER_COMPOSE down --remove-orphans > /dev/null 2>&1 || true

# Explicitly pass build args to ensure variable substitution works
if DOCKERFILE=$DOCKERFILE IMAGE_TAG=$IMAGE_TAG $DOCKER_COMPOSE build; then
    print_success "Docker image built successfully"
else
    print_error "Failed to build Docker image"
    exit 1
fi

print_header "Step 4: Starting Services"

print_info "Starting phntm_bridge container..."
if DOCKERFILE=$DOCKERFILE IMAGE_TAG=$IMAGE_TAG $DOCKER_COMPOSE up -d; then
    print_success "Container started successfully"
else
    print_error "Failed to start container"
    exit 1
fi

# Wait for services to start
print_info "Waiting for services to initialize (30 seconds)..."
sleep 30

# Check if container is still running
if docker ps | grep -q phntm-bridge; then
    print_success "Container is running"
else
    print_error "Container failed to start. Check logs with: docker logs phntm-bridge"
    exit 1
fi

# Verify chat server
print_info "Verifying chat server..."
if docker exec phntm-bridge curl -s http://localhost:3080/health > /dev/null 2>&1; then
    print_success "Chat server is running"
else
    print_warning "Chat server may not be responding yet. It should start shortly."
fi

print_header "Setup Complete!"

echo "Your Phantom Bridge Client is now ready to use!"
echo ""
echo "Robot ID: $ROBOT_ID"
echo ""
echo "Next steps:"
echo "1. Open your browser and navigate to:"
echo -e "   ${YELLOW}https://bridge.phntm.io/$ROBOT_ID${NC}"
echo ""
echo "2. Chat widgets (Spot, Drone, Operator) are available in the Widgets menu"
echo "   Each widget supports detection prompts via Grounding DINO"
echo ""
echo "3. Check the bridge logs (if needed):"
echo -e "   ${YELLOW}docker logs phntm-bridge${NC}"
echo ""
echo "4. To stop the bridge:"
echo -e "   ${YELLOW}$DOCKER_COMPOSE down${NC}"
echo ""
echo "5. To restart the bridge:"
echo -e "   ${YELLOW}$DOCKER_COMPOSE up -d${NC}"
echo ""
echo "Configuration file location: ${YELLOW}$HOME/phntm_bridge.yaml${NC}"
echo "Project directory: ${YELLOW}$SCRIPT_DIR${NC}"
echo ""
echo "For more information, see: ${SCRIPT_DIR}/README.md"
echo ""
