idf_component_register(SRCS "can_msg_queue.c" "web.cpp" "wifi.c" "main.cpp" "joints_storage.c" "state_machine.cpp" "communication_handler.cpp" "Joint.cpp" "Arm.cpp"
                    INCLUDE_DIRS "."
                    EMBED_FILES "www/index.html")