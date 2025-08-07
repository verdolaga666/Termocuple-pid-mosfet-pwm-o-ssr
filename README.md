# Termocuple-pid-mosfet-pwm-o-ssr
Termocuple con control PiD por mosfet pwm(o ssr)
Controlador de Temperatura Digital Universal
Este es un proyecto de código abierto para un controlador de temperatura digital robusto y versátil, ideal para aplicaciones como cerveceras caseras, hornos de reflujo, control de acuarios, o cualquier sistema que requiera una gestión precisa del calor. Basado en una placa Arduino, este firmware combina una interfaz de usuario intuitiva con funcionalidades avanzadas, utilizando un encoder rotatorio y una pantalla OLED para una experiencia de usuario completa.

========================================================
*Thermocouple with PID Control via MOSFET PWM (or SSR)
This is an open-source project for a robust and versatile digital temperature controller, ideal for applications such as homebrewing, reflow ovens, aquarium control, or any system that requires precise heat management. Based on an Arduino board, this firmware combines an intuitive user interface with advanced functionalities, using a rotary encoder and an OLED screen for a complete user experience.*


Características Principales:

Modos de Control Versátiles:

Control PID (Proporcional-Integral-Derivativo): Para una precisión milimétrica en la regulación de la temperatura, ideal para procesos que requieren estabilidad.
Control ON/OFF: Sencillo y efectivo para aplicaciones donde no se necesita alta precisión, con histéresis configurable para evitar ciclos de encendido/apagado constantes.
Control Manual: Permite ajustar la potencia de salida directamente, útil para pruebas y aplicaciones personalizadas.

Gestión de Salida Flexible:

Salida MOSFET con PWM: Control de potencia preciso y eficiente para elementos calefactores de CC.
Salida para SSR (Solid State Relay): Compatible con relés de estado sólido para controlar cargas de alta potencia o CA.
Interfaz de Usuario Intuitiva:
Pantalla OLED (128x64): Muestra de forma clara la temperatura actual, el punto de ajuste (setpoint), el estado del sistema y los modos de control.
Encoder Rotatorio con Botón: Facilita la navegación por los menús y el ajuste rápido de los valores, incluso con guantes.


Funciones de Protección y Seguridad:

Alarma de Alta Temperatura: Alerta visual y sonora cuando la temperatura excede un límite configurable, previniendo daños y accidentes.
Detección de Fallo de Calefacción: El sistema detecta si la temperatura no sube como se espera en un período de tiempo, activando un estado de error.
Temporizador de Inactividad: Vuelve automáticamente a la pantalla de control después de un tiempo sin interacción, evitando configuraciones accidentales.

Configuración y Persistencia:

Memoria EEPROM: Guarda todas las configuraciones (setpoint, PID, histéresis, calibración, etc.) para que persistan incluso después de un reinicio.
Calibración de Sensor: Permite ajustar un offset de temperatura para corregir cualquier desviación del sensor.


Componentes Clave del Hardware:

Microcontrolador: Arduino (o similar).
Sensor de Temperatura: Termopar tipo K con módulo MAX6675.
Pantalla: OLED U8x8_SSD1306 (I2C).
Interfaz de Usuario: Encoder rotatorio con botón.
Salida: MOSFET (para PWM) o Salida Digital para SSR.


Cómo Usar este Código:

Clona o descarga este repositorio.
Instala las librerías necesarias (listadas en el código).
Carga el sketch en tu placa Arduino.

Conecta los componentes según la configuración de pines definida en el archivo .ino.

Este proyecto es una excelente base para cualquier persona interesada en la automatización, electrónica o control de procesos. ¡Siéntete libre de explorarlo, modificarlo y adaptarlo a tus necesidades!
