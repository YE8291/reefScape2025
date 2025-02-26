# FRC REEFSCAPE 2024-2025

Codigo de la temporada reefscape (2025), desarrollado en Java.


## Subsistemas
Los subsistemas desarrollados para esta temporada son 3 actualmente iniciando con el desarrollo de un cuarto subsistema,
estos subsistemas son los siguientes:

>  ### Drivetrain:
    Este subsistema se encarga del manejo de la transmision del robot, la transmision usada en el robot es de tipo tank drive,
    se implementa simulacion en este subsistema para simular el movimiento del robot y a futuro realizar las pruebas de autonomo
    durante su desarrollo. Haciendo uso de encoders como sensor para el movimiento.

>  ### Elevator:
    Este susbsistema se encarga del control del elevador utilizado para subir los mecanismos hasta la altura deseada y poder anotar
    los numeros de juego en su zona. Este subsistema debe de implementar un controlador PID para el movimiento preciso, por lo que
    requerira del uso de sensores principalmente encoders.

> ### Shooter:
    Este subsitema se encargara del control del elevador utilizado para interactuar con los elementos de juego, principalmente para
    recogerlos y anotarlos, este utilizara un sensor para determinar si contiene un elemento de juego o no.

## Comandos
Los comandos desarrollados para esta temporada se agregara despues su descripcion aqui.

#### Notas adicionales:
    Este año se busca trabajar la parte de la simulacion para realizacion de pruebas tanto en mecanismos y rutas de autonomo.
    Para autonomo se utilizara path planner
    Se busca implementar controladores PID a todo lo que lo requiera
    Posteriormente se realizara una optimizacion del codigo y se realizara su respectiva documentacion

## Git branches:
El proyecto se divide en tres ramas:
> Master: Rama principal contiene el ultimo codigo estable (funcionamiento comprobado de forma fisica)

> Main: Rama secundaria contiene el codigo en desarrollo que se probara en el robot

> Simulation: Rama secundaria a master que contiene el codigo que se utiliza para trabajar simulaciones