Sequência de desenvolvimento:

1. acelerômetro_1

A função deste código é escrever e ler em um dos registradores do acelerômetro para validação da interface SPI
de comunicação entre MSP430 e o acelerômetro.

2. acelerômetro_2

Este código possibilita a verificação do comportamento do carro através de LEDs. Dependendo da inclinação do
acelerômetro (para frente e/ou esquerda ou direita) os LEDs que indicam os movimentos serão acesos ou apagados.
Quando aceso, o LED vermelho indica que o carro se desloca para frente. O acendimento de qualquer LED verde ao
mesmo tempo que o vermelho indica uma trajetória circular no sentido horário ou anti-horário. O apagamento do
LED vermelho em qualquer situação indica repouso do carro.

3. acelerômetro_3

Este código é semelhante ao acelerômetro_1. Nesse caso, a comunicação SPI é realizada a 3 fios e as funções de
leitura e escrita foram modificadas.

4. acelerômetro_4

Este código é semelhante ao acelerômetro_2. Nesse caso, a comunicação SPI é realizada a 3 fios e as funções de
leitura e escrita foram modificadas.

5. rf_1

Este código faz parte de um código ainda mais complexo desenvolvido pela Texas Instruments. O código original,
que não encontra-se no repositório, foi escrito para atender ao uso de diferentes plataformas do fabricante.
Por isso, houve a necessidade de editar o código para diminuir o número de arquivos, condensando-os em apenas
3 arquivos. Após a edição, esse código consta de 1 arquivo .h contendo declarações, 1 arquivo .c contendo
funções, e 1 arquivo .c contendo a função principal e a rotina de interrupção.

6. rf_2

Este código é uma alteração do rf_1. As alterações foram realizadas de modo a deixar o código mais legível.
Foram adotados os mesmos padrões de escrita dos códigos de validação do acelerômetro. O algoritmo do código
desenvolvido pela Texas Instruments foi mantido.

7. integração_1

Este código foi adaptado de modo a manter em funcionamento simultâneo o acelerômetro e o RF. Desse modo, os
leds LED1 e LED2 acendem se a comunicação MSP430 - ACELERÔMETRO e MSP430 - RF forem estabelecidas,
respectivamente. Após o acendimento desses leds é enviado um pacote por RF cuja função é acender um LED verde
em outra placa onde deve estar gravado o código rf_2.