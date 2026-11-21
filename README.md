# Regional — Festival de Robótica SESI/SENAI

Código dos mecanismos do robô para o **Festival de Robótica SESI/SENAI**. Cada mecanismo é um projeto WPILib separado, para poder testar um de cada vez.

| Pasta | Mecanismo | Motores |
|---|---|---|
| `Garra/` e `Garra/Garra/` | Garra/intake (`GarraIntakeSubsystem`) | SparkMax e TalonFX |
| `Elevador/Mecanismos/Elevador/` | Elevador (`ElevatorSubsystem`) | SparkMax |

## Tecnologias

Java 17 · WPILib 2026 · REVLib · Phoenix 6 · [YAMS](https://github.com/Yet-Another-Software-Suite/YAMS)

## Como rodar

1. Abra a pasta do mecanismo (por exemplo `Garra/Garra`) no **WPILib VS Code**.
2. Compile com `./gradlew build`.
3. Faça o deploy com `./gradlew deploy`.

O código integrado com o swerve está em [FRC-CODE-MS-main](https://github.com/nicolasmneves/FRC-CODE-MS-main).
