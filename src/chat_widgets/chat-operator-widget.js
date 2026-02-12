import { CompositePanelWidgetBase } from 'https://bridge.phntm.io/static/widgets/inc/composite-widget-base.js'

export class ChatOperatorWidget extends CompositePanelWidgetBase {

    static LABEL = "Chat: Operator";

    static DEFAULT_WIDTH = 4;
    static DEFAULT_HEIGHT = 12;

    constructor(panel) {
        super(panel, 'chat-operator');

        // Get custom config if available
        let default_chat_topic = this.client.getConfigParam('chat_operator.topic') || '/operator/chat';

        // Chat history storage
        this.messages = [];
        this.maxMessages = 100;
        this.storageKey = 'chat_operator_messages';
        this.promptsStorageKey = 'chat_operator_active_prompts';

        // Active detection prompts
        this.activePrompts = [];
        this.loadPromptsFromStorage();

        // Load previous chat history
        this.loadMessagesFromStorage();

        // Create UI elements
        this.container_el = $('<div class="chat-container"></div>');
        this.messages_el = $('<div class="chat-messages"></div>');
        this.prompts_el = $('<div class="prompt-list operator-prompts"></div>');
        this.input_container_el = $('<div class="chat-input-container"></div>');
        this.input_el = $('<input type="text" class="chat-input" placeholder="Detection prompt (e.g. red ball)..."/>');
        this.send_btn = $('<button class="chat-send-btn">Send</button>');

        this.input_container_el.append(this.input_el, this.send_btn);
        this.container_el.append(this.messages_el, this.prompts_el, this.input_container_el);
        this.widget_el.append(this.container_el);

        // Restore chat history to UI
        this.restoreMessagesToUI();
        this.renderPrompts();

        // Set up topic source for receiving messages
        this.sources.add(
            "std_msgs/msg/String",
            "Chat messages from Operator",
            default_chat_topic,
            1,
            (topic, msg) => this.onChatMessage(topic, msg),
        );

        this.sources.loadAssignedTopicsFromPanelVars();

        // Open write channel for sending prompts back to the chat topic
        this.chatTopic = default_chat_topic;
        this.client.openWriteChannel(this.chatTopic, 'std_msgs/msg/String');

        // Bind events
        let that = this;
        this.send_btn.click(() => that.sendMessage());
        this.input_el.keypress((ev) => {
            if (ev.which === 13) { // Enter key
                that.sendMessage();
            }
        });

        // Add welcome message only if no history exists
        if (this.messages.length === 0) {
            this.addMessage('system', 'Connected to Operator chat interface');
        }
    }

    onChatMessage(topic, msg) {
        if (this.panel.paused) return;
        this.panel.updateFps();

        // Try to parse as JSON detection result
        try {
            let data = JSON.parse(msg.data);
            if (data.type === 'detection') {
                this.renderDetection(data);
                return;
            }
        } catch (e) {
            // Not JSON, render as normal text
        }

        this.addMessage('operator', msg.data);
    }

    renderDetection(data) {
        const timestamp = new Date().toLocaleTimeString();
        const detections = data.detections || [];
        const cameraShort = (data.camera || '').split('/').pop() || data.camera;

        // Build detection info text
        let infoText = detections.map(d =>
            `${d.label} (${(d.confidence * 100).toFixed(0)}%)`
        ).join(', ');

        // Create message element with image
        let msg_el = $('<div class="chat-message bot-message detection-message"></div>');
        msg_el.append($(`<span class="message-time">${timestamp}</span>`));
        msg_el.append($(`<span class="message-sender">dino:</span>`));

        if (data.image_base64) {
            let img = $(`<img class="detection-image" src="data:image/jpeg;base64,${data.image_base64}" />`);
            msg_el.append(img);
        }

        msg_el.append($(`<div class="detection-info">${this.escapeHtml(cameraShort)}: ${this.escapeHtml(infoText)}</div>`));

        this.messages_el.append(msg_el);
        this.messages_el.scrollTop(this.messages_el[0].scrollHeight);

        // Save a summary to history (without the base64 image to save storage)
        const message = { sender: 'operator', text: `[Detection] ${infoText} (${cameraShort})`, timestamp };
        this.messages.push(message);
        if (this.messages.length > this.maxMessages) {
            this.messages.shift();
        }
        this.saveMessagesToStorage();
    }

    addMessage(sender, text) {
        const timestamp = new Date().toLocaleTimeString();
        const message = { sender, text, timestamp };

        this.messages.push(message);
        if (this.messages.length > this.maxMessages) {
            this.messages.shift();
        }

        // Save to localStorage
        this.saveMessagesToStorage();

        const senderClass = sender === 'user' ? 'user-message' :
                           sender === 'operator' ? 'bot-message' : 'system-message';

        const msg_el = $(`<div class="chat-message ${senderClass}">
            <span class="message-time">${timestamp}</span>
            <span class="message-sender">${sender}:</span>
            <span class="message-text">${this.escapeHtml(text)}</span>
        </div>`);

        this.messages_el.append(msg_el);
        this.messages_el.scrollTop(this.messages_el[0].scrollHeight);
    }

    sendMessage() {
        const text = this.input_el.val().trim();
        if (!text) return;

        this.input_el.val('');

        // Send as detection prompt via write channel
        const promptPayload = JSON.stringify({ type: 'add_prompt', prompt: text });
        this.client.writeTopicData(this.chatTopic, { data: promptPayload });

        // Add to active prompts list
        if (!this.activePrompts.includes(text)) {
            this.activePrompts.push(text);
            this.savePromptsToStorage();
            this.renderPrompts();
        }

        this.addMessage('system', `Added detection prompt: ${text}`);
    }

    removePrompt(promptText) {
        const removePayload = JSON.stringify({ type: 'remove_prompt', prompt: promptText });
        this.client.writeTopicData(this.chatTopic, { data: removePayload });

        this.activePrompts = this.activePrompts.filter(p => p !== promptText);
        this.savePromptsToStorage();
        this.renderPrompts();

        this.addMessage('system', `Removed detection prompt: ${promptText}`);
    }

    renderPrompts() {
        this.prompts_el.empty();
        if (this.activePrompts.length === 0) return;

        for (const prompt of this.activePrompts) {
            let chip = $('<span class="prompt-chip operator-prompt-chip"></span>');
            chip.append($(`<span class="prompt-text">${this.escapeHtml(prompt)}</span>`));
            let removeBtn = $('<span class="prompt-remove">&times;</span>');
            removeBtn.click(() => this.removePrompt(prompt));
            chip.append(removeBtn);
            this.prompts_el.append(chip);
        }
    }

    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }

    saveMessagesToStorage() {
        try {
            localStorage.setItem(this.storageKey, JSON.stringify(this.messages));
        } catch (e) {
            console.warn('Failed to save chat history to localStorage:', e);
        }
    }

    loadMessagesFromStorage() {
        try {
            const stored = localStorage.getItem(this.storageKey);
            if (stored) {
                this.messages = JSON.parse(stored);
            }
        } catch (e) {
            console.warn('Failed to load chat history from localStorage:', e);
        }
    }

    savePromptsToStorage() {
        try {
            localStorage.setItem(this.promptsStorageKey, JSON.stringify(this.activePrompts));
        } catch (e) {
            console.warn('Failed to save prompts to localStorage:', e);
        }
    }

    loadPromptsFromStorage() {
        try {
            const stored = localStorage.getItem(this.promptsStorageKey);
            if (stored) {
                this.activePrompts = JSON.parse(stored);
            }
        } catch (e) {
            console.warn('Failed to load prompts from localStorage:', e);
        }
    }

    restoreMessagesToUI() {
        for (const message of this.messages) {
            const senderClass = message.sender === 'user' ? 'user-message' :
                               message.sender === 'operator' ? 'bot-message' : 'system-message';

            const msg_el = $(`<div class="chat-message ${senderClass}">
                <span class="message-time">${this.escapeHtml(message.timestamp)}</span>
                <span class="message-sender">${this.escapeHtml(message.sender)}:</span>
                <span class="message-text">${this.escapeHtml(message.text)}</span>
            </div>`);

            this.messages_el.append(msg_el);
        }
        this.messages_el.scrollTop(this.messages_el[0].scrollHeight);
    }

    setupMenu(menu_els) {
        super.setupMenu(menu_els);

        // Clear chat button
        let clear_line_el = $('<div class="menu_line"></div>');
        let clear_btn = $('<a href="#" id="clear_chat_' + this.panel.n + '">Clear Chat</a>');
        clear_btn.appendTo(clear_line_el);

        let that = this;
        clear_btn.click((ev) => {
            ev.preventDefault();
            that.clearChat();
        });
        menu_els.push(clear_line_el);
    }

    clearChat() {
        this.messages = [];
        this.messages_el.empty();
        this.saveMessagesToStorage();
        this.addMessage('system', 'Chat cleared');
    }

    onResize() {
        // Handle resize if needed
    }

    onClose() {
        console.log('Closing Chat: Operator widget');
        super.onClose();
    }
}
