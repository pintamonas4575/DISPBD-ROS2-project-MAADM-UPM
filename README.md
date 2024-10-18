# DISPBD-ROS2-project_MAADM-UPM

Creación de un nodo personalizado en ROS2 Humble para controlar la tortuga "turtlesim". Este es el paquete que deberá ir dentro del workspace de ROS2.

**Funcionalidades:**

- Moverse pulsando las **teclas W (arriba), S (abajo), A (izquierda) y D (derecha)**.
- La pulsación de la **tecla SPACE** habilita/dehabilita el pintado, de tal manera que cuando está habilitado la tortuga pinta en las posiciones en las que se encuentre.
- Cuando se pulsa la **tecla C** (clear) se borra todo lo dibujado.
- Cuando se pulsa la **tecla R** (reset) se reinicia la posición de la tortuga al 
centro de la pantalla.
- Si se presiona **cualquier otra tecla** la tortuga se para.
- Posibilidad de modificar **dos parámetros**: **"speed" y "log_level"**. Son la velocidad y el nivel de log de los mensajes respectivamente.

# 📂 Carpeta *"launch"*

Contiene un launcher que **no** funciona correctamente.

# 📂 Carpeta *"tortuga_loca"*

Contiene las versiones incrementales del nodo implementado, siendo el archivo que comienza por 'f_...' el final.

# 📃 Archivo *"setup.py"*

Configuraciones para el paquete y los nodos. Cada vez que se modifique se debe hacer *colcon build*.

# Ejecución y pruebas

Para probar se deben tener 3 terminales y ejecutar 3 comandos distintos:

1. ros2 run turtlesim turtlesim_node
2. ros2 run tortuga_loca f_nodo_tortuga_log
3. ros2 param set /f_nodo_tortuga_log <speed/log_level> <nuevo_valor> 

Para *speed*, los valores son float (ej: 5.0).

Para *log_level*, los valores son string mayúsculas de los niveles de log (DEBUG, INFO, WARN, ERROR, FATAL).

# ⚖️ Licencia 

No da la suscripción de personas para leer la publicación diaria del BOE.

# 👤 Contacto

Cualquier duda o sugerencia contactar con el autor:

Alejandro Mendoza: alejandro.embi@gmail.com