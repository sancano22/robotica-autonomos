## SW de simulación
Webots

## Descripción del Proyecto

El presente proyecto tiene como objetivo el desarrollo e implementación de un robot móvil autónomo en el simulador Webots, utilizando control cinemático diferencial y sensores para la percepción del entorno.

**El robot es capaz de:**

    - Detectar y evitar obstáculos en tiempo real mediante la combinación de un sensor LIDAR 2D y sensores de distancia (ultrasónicos o infrarrojos).

    - Construir un mapa local del entorno (grilla de ocupación 8x8) a partir de los datos del LIDAR.

    - Planificar rutas óptimas en el mapa utilizando el algoritmo de planificación A* (A-Star).

    - Navegar de manera autónoma hacia un objetivo definido en el entorno simulado, ajustando su trayectoria en función de la percepción actualizada.

El desarrollo integra percepción, planificación y control en un entorno dinámico, permitiendo que el robot reaccione ante cambios en su entorno y tome decisiones en tiempo real. El robot se prueba en un entorno controlado de 4m x 4m con obstáculos distribuidos, evaluando su desempeño en términos de eficiencia de navegación, precisión de planificación y robustez en la evasión de obstáculos.