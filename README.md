# Robot Sube-Escaleras

Proyecto de la asignatura **Mecatrónica** del **Grado en Ingeniería de Robótica Software** de la **Universidad Rey Juan Carlos (URJC)**.

## Componentes del grupo:

- Adrián Manzanares Romera: [Amanza17](https://github.com/Amanza17)
- Claudia Élez Mencía: [celezm](https://github.com/celezm)
- Mario Casero Sández: [MarioC05](https://github.com/MarioC05)


## Objetivo

Diseñar, fabricar y programar un robot terrestre capaz de **subir al menos un peldaño** de las escaleras del laboratorio III de la URJC, manteniendo la estabilidad y evitando obstáculos durante la aproximación.

### Requisitos principales

- Subir al menos un peldaño en **menos de 1 minuto**.
- Detectar y evitar obstáculos durante la aproximación, partiendo de **20 cm de la contrahuella**, dentro del tiempo establecido.
- No superar **25 × 25 × 25 cm** para el robot completo.
- Utilizar **baterías recargables**, sin alimentación por cable durante las pruebas.
- Diseñar y fabricar una **estructura propia**.
- Incorporar una interfaz para **iniciar y detener** el funcionamiento.
- Respetar la restricción de **locomoción sin ruedas** establecida en el enunciado.

## Diseño seleccionado

Durante la fase de diseño conceptual se estudiaron cuatro alternativas:

1. Robot cuadrúpedo tipo perro.
2. Robot bípedo con cuerpo horizontal y rueda loca delantera retráctil.
3. Robot serpiente modular.
4. Robot lineal tipo impresora 3D, con pie fijo y pie móvil.

La propuesta elegida para continuar el desarrollo es el **bípedo**. Consta de dos patas articuladas y un apoyo delantero mediante rueda loca que puede replegarse hacia atrás.

## Estructura del repositorio


```text
Mecatronica/
├── README.md
├── planificacion/
│   ├── diagrama_gantt
│   └── diagrama_pert
├── diseno_conceptual/
│   │   ├── matriz_calidad
│   │   └── matriz_seleccion
│   │   └── diseño_conceptual
```
## Estado del proyecto

Actualmente se ha realizado la evaluación conceptual de alternativas y se ha seleccionado el bípedo para continuar con el diseño detallado. 

Están pendientes la finalización del diseño, validación del mecanismo, el dimensionamiento definitivo de los motores, la integración y las pruebas físicas.
