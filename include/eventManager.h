/*
 *  Bus d'événements singleton avec queue circulaire et registre de callbacks
 *
 *  @author BOUFALLOUS Amin
 *  @modified 2026-04-11
 */

#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include "types.h"
#include <cstring>

#define EVENT_QUEUE_SIZE 16
#define MAX_CALLBACKS 8

// ========== TEMPLATE CALLBACK ==========
template<typename T>
class Callback {
public:
    typedef void (*function_t)(const T*);
    
    Callback() : func(nullptr) {}
    Callback(function_t f) : func(f) {}
    
    void call(const T* data) {
        if(func) func(data);
    }

    bool isSet() const { return func != nullptr; }

private:
    function_t func;
};

// ========== EVENT MANAGER SINGLETON ==========
class EventManager {
public:
    // Singleton access
    static EventManager& getInstance() {
        static EventManager instance;
        return instance;
    }
    
    // Post event (non-blocking)
    void postEvent(const event_t& event) {
        if(queue_count >= EVENT_QUEUE_SIZE) return;
        
        event_queue[queue_tail] = event;
        queue_tail = (queue_tail + 1) % EVENT_QUEUE_SIZE;
        queue_count++;
    }
    
    // Process all events
    void processEvents() {
        while(queue_count > 0) {
            event_t event = event_queue[queue_head];
            queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;
            queue_count--;
            
            last_event = event;
            
            // Call all subscribed callbacks for this event type
            for(uint8_t i = 0; i < MAX_CALLBACKS; i++) {
                if(callbacks[event.type][i].isSet()) {
                    callbacks[event.type][i].call(&event);
                }
            }
        }
    }
    
    // Subscribe to event (template version for type safety)
    template<typename T>
    void subscribe(event_type_t type, void (*callback)(const T*)) {
        if(type >= 16) return;
        
        for(uint8_t i = 0; i < MAX_CALLBACKS; i++) {
            if(!callbacks[type][i].isSet()) {
                callbacks[type][i] = Callback<event_t>((void (*)(const event_t*))callback);
                return;
            }
        }
    }

    // Simple subscribe (event-based)
    void subscribe(event_type_t type, Callback<event_t>::function_t callback) {
        if(type >= 16) return;

        for(uint8_t i = 0; i < MAX_CALLBACKS; i++) {
            if(!callbacks[type][i].isSet()) {
                callbacks[type][i] = Callback<event_t>(callback);
                return;
            }
        }
    }
    
    event_t getLastEvent() { return last_event; }
    void clearQueue() { queue_count = 0; queue_head = 0; queue_tail = 0; }
    
private:
    EventManager() : queue_head(0), queue_tail(0), queue_count(0) {
        memset(&last_event, 0, sizeof(event_t));
    }
    
    event_t event_queue[EVENT_QUEUE_SIZE];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    event_t last_event;
    
    Callback<event_t> callbacks[16][MAX_CALLBACKS];
};

#endif
