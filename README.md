# Vyde Drone Simulation

Simulador de quadrotor en Rust, Bevy y Avian. El control y la física se ejecutan a **240 Hz**, independientemente de los FPS de renderizado.

## Ejecutar

```sh
cargo run -p drone_sim

# Sin ventana: el controlador autónomo mantiene (0, 2, 0).
HEADLESS=1 cargo run -p drone_sim
```

## Controles

| Tecla | Acción |
| --- | --- |
| W / S | Avanzar / retroceder según el rumbo FPV |
| A / D | Desplazarse a izquierda / derecha, sin girar |
| Espacio / Shift | Subir / bajar |
| Q / E | Girar el rumbo |
| Ctrl+F | Alternar FPV y cámara libre |
| R | Reiniciar posición, velocidades, motores y memoria del estabilizador |

Al soltar las teclas, el drone frena y mantiene posición, altura y rumbo. En cámara libre, las teclas mueven la cámara y el drone mantiene su posición actual. Los reinicios no recargan la batería.

## Modelo de vuelo

- **Estabilizador:** error de actitud por cuaterniones en coordenadas del cuerpo, amortiguación angular escalada por inercia, lazos de posición y velocidad, compensación de inclinación e integral limitada para rechazar viento sostenido.
- **Motores:** mezcla según geometría y coeficientes individuales; saturación que prioriza el empuje colectivo, retardo de respuesta, límite de variación de RPM y techo de velocidad según tensión disponible.
- **Cuerpo rígido:** Avian integra fuerzas y torques, gravedad, inercia y movimiento giroscópico. Las colisiones usan el collider del drone y la misma geometría en modo gráfico y headless; el suelo permite aterrizar y los contactos con obstáculos provocan reinicio.
- **Aerodinámica:** resistencia lineal y cuadrática según velocidad relativa al aire, densidad, amortiguación angular y pequeño momento aerodinámico respecto al centro de masas. Efecto suelo gradual según altura e inclinación.
- **Viento:** velocidad constante, ráfagas suaves con dirección y periodo configurables, y turbulencia reproducible con variación espacial. Hay viento moderado por defecto.
- **Batería:** potencia de rotor proporcional a `Q·ω`, pérdidas eléctricas, caída de tensión bajo carga, interpolación de la curva de descarga y respuesta térmica gradual.
- **Red:** latencia, jitter y pérdida de paquetes configurables para comandos y telemetría; el estabilizador interno sigue funcionando al ritmo de la física.

Es un modelo de vuelo aproximado, no CFD ni un modelo calibrado para un drone real. La turbulencia usa señales suaves deterministas, no un modelo meteorológico. Actualmente se soporta un solo drone. Los parámetros avanzados declarados en configuración no están todos implementados; por ejemplo, flexión de brazos y distorsión de cámara.

## Configuración

Los parámetros están en `simulation/src/config.rs`:

- `DroneConfig`: masa, inercia, motores, resistencia y efecto suelo.
- `EnvironmentConfig`: gravedad, densidad y viento. Velocidades en m/s, duración de ráfagas en segundos y longitud de turbulencia en metros.
- `BatteryConfig`, `SensorConfig`, `NetworkConfig`, `DomainRandomizationConfig`, `CameraConfig`, `SimConfig`.

Para aire en calma, poner a cero `wind_constant`, `wind_gust_intensity` y `turbulence_intensity`. La aleatorización de dominio y la simulación de red están desactivadas por defecto.

`DroneController` permite implementar controladores autónomos mediante comandos de posición, velocidad, actitud, empuje/torque o RPM. `DroneCommand::None` significa mantener la posición actual; para cortar motores se utiliza `MotorCommand { rpms: [0.0; 4] }`.

## Validación

```sh
cargo test -p drone_sim
```

Incluye pruebas unitarias de controles, mezcla, motores, viento, aerodinámica y batería, además de vuelos headless con la física real: recuperación de inclinación, hover con viento y latencia, frenado y mantenimiento de posición, aterrizaje sin motores, choques, reinicios e independencia respecto a los FPS.
