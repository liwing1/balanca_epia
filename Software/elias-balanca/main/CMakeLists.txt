idf_component_register(SRCS "main.c"
                    REQUIRES 
                        logic_analyzer
                        MCP3564
                        driver
                    INCLUDE_DIRS "."
)