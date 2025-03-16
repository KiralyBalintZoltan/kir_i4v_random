cmake_minimum_required(VERSION 3.5)
project(kir_i4v_random)

# Kötelező ROS2 komponensek megtalálása
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python kód telepítése a setup.py alapján
ament_python_install_package(${PROJECT_NAME})

# A launch fájlok telepítése
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# A képfájlok vagy egyéb statikus erőforrások telepítése (opcionális)
install(
    DIRECTORY img
    DESTINATION share/${PROJECT_NAME}/img
)

# Az erőforrások (pl. ikonok, statikus fájlok) telepítése
install(
    DIRECTORY resource
    DESTINATION share/${PROJECT_NAME}/resource
)

# A dokumentációs fájlok telepítése (opcionális)
install(
    FILES README.md LICENSE
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
