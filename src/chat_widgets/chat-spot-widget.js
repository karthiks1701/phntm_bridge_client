import { CompositePanelWidgetBase } from 'https://bridge.phntm.io/static/widgets/inc/composite-widget-base.js'

export class ChatSpotWidget extends CompositePanelWidgetBase {

    static LABEL = "Chat: Spot";

    static DEFAULT_WIDTH = 4;
    static DEFAULT_HEIGHT = 12;

    constructor(panel) {
        super(panel, 'chat-spot');

        // Get custom config if available
        let default_chat_topic = this.client.getConfigParam('chat_spot.topic') || '/spot/chat';
        
        // Chat history storage
        this.messages = [];
        this.maxMessages = 100;
        this.storageKey = 'chat_spot_messages';
        
        // Load previous chat history
        this.loadMessagesFromStorage();

        // Create UI elements
        this.container_el = $('<div class="chat-container"></div>');
        this.messages_el = $('<div class="chat-messages"></div>');
        this.input_container_el = $('<div class="chat-input-container"></div>');
        this.input_el = $('<input type="text" class="chat-input" placeholder="Message Spot..."/>');
        this.send_btn = $('<button class="chat-send-btn">Send</button>');
        
        this.input_container_el.append(this.input_el, this.send_btn);
        this.container_el.append(this.messages_el, this.input_container_el);
        this.widget_el.append(this.container_el);

        // Restore chat history to UI
        this.restoreMessagesToUI();

        // Set up topic source for receiving messages
        this.sources.add(
            "std_msgs/msg/String",
            "Chat messages from Spot",
            default_chat_topic,
            1,
            (topic, msg) => this.onChatMessage(topic, msg),
        );

        this.sources.loadAssignedTopicsFromPanelVars();

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
            this.addMessage('system', 'Connected to Spot chat interface');
        }
    }

    onChatMessage(topic, msg) {
        if (this.panel.paused) return;
        this.panel.updateFps();
        
        this.addMessage('spot', msg.data);
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
                           sender === 'spot' ? 'bot-message' : 'system-message';
        
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

        this.addMessage('user', text);
        this.input_el.val('');

        // Send message via data channel if available
        const topics = this.sources.getAssignedTopics();
        if (topics.length > 0) {
            const topic = topics[0];
            this.client.sendTopicData(topic, { data: text });
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

    restoreMessagesToUI() {
        for (const message of this.messages) {
            const senderClass = message.sender === 'user' ? 'user-message' : 
                               message.sender === 'spot' ? 'bot-message' : 'system-message';
            
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
        console.log('Closing Chat: Spot widget');
        super.onClose();
    }
}
