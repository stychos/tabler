idf_component_register(
    SRCS "rc522.c" "rc522_picc.c" "rc522_mifare.c" "rc522_ultralight.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES driver
)
