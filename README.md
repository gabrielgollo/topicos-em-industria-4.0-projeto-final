# Projeto de Controle de Ventoinha com Arduino e Node.js

Este repositório contém todos os artefatos produzidos durante o semestre no contexto do projeto de controle de uma ventoinha utilizando um Arduino UNO em comunicação com um servidor Node.js via Serial. A seguir, estão detalhadas as seções da documentação:

## 1. Apresentação do Problema Escolhido

O problema abordado neste projeto é de realizar o controle eficiente da velocidade de uma ventoinha por meio de um Arduino UNO.


## 2. Proposta de Solução

A solução proposta consiste em utilizar um Arduino UNO para controlar a ventoinha por meio do sinal PWM, enquanto monitora a rotação por meio de um tacômetro. Além disso, foi implementado um controlador PID para ajuste dinâmico da velocidade. A comunicação entre o Arduino e um servidor Node.js é realizada via Serial, permitindo a visualização e controle remoto.

![Diagrama de conexão do arduino com a ventoinha](https://github.com/gabrielgollo/topicos-em-industria-4.0-projeto-final/blob/main/DIAGRAMA.png)

![Construção do projeto](https://github.com/gabrielgollo/topicos-em-industria-4.0-projeto-final/blob/main/Constru%C3%A7%C3%A3o%20do%20projeto.jpeg)

## 3. Resultados

Para visualizar o resultado final do projeto, consulte as imagens nas Figuras 1, 2 e 3 anexadas a esta documentação manualmente.


![Figura 1.](https://github.com/gabrielgollo/topicos-em-industria-4.0-projeto-final/blob/main/1.png)

![Figura 2.](https://github.com/gabrielgollo/topicos-em-industria-4.0-projeto-final/blob/main/2.png)

![Figura 3.](https://github.com/gabrielgollo/topicos-em-industria-4.0-projeto-final/blob/main/3.png)

# Execução do projeto

A versão mais recente do projeto está na pasta `v3_fan_control` e utiliza o framework Platform.io com Arduino para o desenvolvimento e gravação do código.

Na pasta `node_server` se encontra um servidor HTTP e um cliente da comunicação com a porta Serial, onde a apartir da página WEB é possível visualizar e mandar comandos para o arduino