Sequência de desenvolvimento:

1. accelerometer_bring-up_1

A função deste código é escrever e ler em um dos registradores do acelerômetro para validação da interface SPI
de comunicação entre MSP430 e o acelerômetro.

2. accelerometer_bring-up_2

Este código possibilita a verificação do comportamento do carro através de LEDs. Dependendo da inclinação do
acelerômetro (para frente e/ou esquerda ou direita) os LEDs que indicam os movimentos serão acesos ou apagados.
Quando aceso, o LED vermelho indica que o carro se desloca para frente. O acendimento de qualquer LED verde ao
mesmo tempo que o vermelho indica uma trajetória circular no sentido horário ou anti-horário. O apagamento do
LED vermelho em qualquer situação indica repouso do carro.

3. rf_1

Este código faz parte de um código ainda mais complexo desenvolvido pela Texas Instruments. O código original,
que não encontra-se no repositório, foi escrito para atender ao uso de diferentes plataformas do fabricante.
Por isso, houve a necessidade de editar o código para diminuir o número de arquivos, condensando-os em apenas
3 arquivos. Após a edição, esse código consta de 1 arquivo .h contendo declarações, 1 arquivo .c contendo
funções, e 1 arquivo .c contendo a função principal e a rotina de interrupção.
