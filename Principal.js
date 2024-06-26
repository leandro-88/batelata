let somColisao;
let somPonto;
let somFundo;
let imgRaquete;
let imgRaqueteOponente;
let imgLataCerveja; // Nova variável para a imagem da lata de cerveja
let toqueY;

let chanceDeErrar = 0;

// Variáveis da bolinha
let xBolinha = 400;
let yBolinha = 300;
let diametroBolinha = 100; // Aumentei o diâmetro da bolinha
let raioBolinha = diametroBolinha / 2;

// Velocidade da bolinha
let velocidadeXBolinha = 3;
let velocidadeYBolinha = 3;
let raqueteDiametro = 120;  // Ajuste o tamanho da raquete
let raqueteRaio = raqueteDiametro / 2;

// Variáveis da raquete
let xRaquete = 10;
let yRaquete = 240;

// Variáveis do oponente
let xRaqueteOponente = 665;
let yRaqueteOponente; // Será ajustado no setup()
let velocidadeYOponente;

let colidiu = false;

// Placar do jogo
let meusPontos = 0;
let pontosDoOponente = 0;

function preload() {
  // Carregando as imagens e sons
  imgRaquete = loadImage('Thadeu.png');  // Imagem da sua raquete
  imgRaqueteOponente = loadImage('Lucca.png');  // Imagem da raquete do oponente
  imgLataCerveja = loadImage('cerveja.png');  // Imagem da lata de cerveja
  somColisao = loadSound('colisao.mp3');
  somPonto = loadSound('ponto.mp3');
  somFundo = loadSound('fundo.mp3');
}

function setup() {
  createCanvas(800, 600);
  somFundo.loop();

  // Posição inicial mais centralizada para a raquete do oponente
  yRaqueteOponente = height / 2 - raqueteDiametro / 2;

  // Inicializa variável de toque
  toqueY = height / 2;
}

function draw() {
  background(0);
  mostraLataCerveja();  // Mostra a lata de cerveja ao invés da bolinha
  movimentaBolinha();
  verificaColisaoBorda();
  mostraRaquete(xRaquete, yRaquete, imgRaquete);  // Sua raquete
  movimentaMinhaRaquete();
  verificaColisaoRaquete(xRaquete, yRaquete);
  mostraRaquete(xRaqueteOponente, yRaqueteOponente, imgRaqueteOponente);  // Raquete do oponente
  movimentaRaqueteOponente();
  verificaColisaoRaquete(xRaqueteOponente, yRaqueteOponente);
  incluiPlacar();
  marcaPonto();
}

function mostraLataCerveja() {
  image(imgLataCerveja, xBolinha, yBolinha, diametroBolinha, diametroBolinha);
}

function movimentaBolinha() {
  xBolinha += velocidadeXBolinha;
  yBolinha += velocidadeYBolinha;
}

function verificaColisaoBorda() {
  if (yBolinha + raioBolinha > height || yBolinha - raioBolinha < 0) {
    velocidadeYBolinha *= -1;
    somColisao.play();
  }

  if (xBolinha + raioBolinha > width) {
    meusPontos += 1;
    somPonto.play();
    reiniciaBolinha();
  } else if (xBolinha - raioBolinha < 0) {
    pontosDoOponente += 1;
    somPonto.play();
    reiniciaBolinha();
  } else if (xBolinha + raioBolinha > width || xBolinha - raioBolinha < 0) {
    // Aumenta a velocidade se a bolinha tocar a borda sem marcar ponto
    if (velocidadeXBolinha > 0) {
      velocidadeXBolinha += 0.5;
    } else {
      velocidadeXBolinha -= 0.5;
    }
    if (velocidadeYBolinha > 0) {
      velocidadeYBolinha += 0.5;
    } else {
      velocidadeYBolinha -= 0.5;
    }
  }
}

function mostraRaquete(x, y, img) {
  // Desenha a imagem da raquete como um círculo
  push();
  ellipseMode(CENTER);
  imageMode(CENTER);
  image(img, x + raqueteRaio, y + raqueteRaio, raqueteDiametro, raqueteDiametro);
  pop();
}

function movimentaMinhaRaquete() {
  // Movimentação da raquete controlada pelo toque
  yRaquete = toqueY;

  // Movimentação da raquete controlada pelas setas do teclado
  if (keyIsDown(UP_ARROW)) {
    yRaquete -= 10;
  }
  if (keyIsDown(DOWN_ARROW)) {
    yRaquete += 10;
  }

  // Constrain para manter a raquete dentro dos limites
  yRaquete = constrain(yRaquete, 5, height - raqueteDiametro - 5);
}

function verificaColisaoRaquete(x, y) {
  colidiu = collideCircleCircle(x + raqueteRaio - 50, y + raqueteRaio -10, raqueteDiametro, xBolinha, yBolinha, diametroBolinha);
  if (colidiu) {
    velocidadeXBolinha *= -1;
    somColisao.play();
  }
}

function movimentaRaqueteOponente() {
  let meioRaquete = raqueteDiametro / 2;
  velocidadeYOponente = yBolinha - yRaqueteOponente - meioRaquete - 30;
  yRaqueteOponente += velocidadeYOponente + chanceDeErrar;
  yRaqueteOponente = constrain(yRaqueteOponente, 5, height - raqueteDiametro - 5);
  calculaChanceDeErrar();
}

function incluiPlacar() {
  stroke(255);
  textSize(15);
  textAlign(CENTER, CENTER);
  fill(color(255, 140, 0));
  
  // Retângulos laranja para o placar
  rect(width / 4 - 12, 10, 25, 20); // Centraliza o primeiro retângulo laranja
  rect(3 * width / 4 - 12, 10, 25, 20); // Centraliza o segundo retângulo laranja
  
  // Texto dentro dos retângulos
  fill(255);
  text(meusPontos, width / 4, 20); // Nome da imagem com a primeira letra maiúscula dentro do retângulo
  text(pontosDoOponente, 3 * width / 4, 20); // Nome da imagem com a primeira letra maiúscula dentro do retângulo
  
  // Nome da raquete fora do retângulo
  textAlign(CENTER);
  text("Véi da Bike", width / 4, 50); // Ajuste a posição conforme necessário
  text("Nega", 3 * width / 4, 50); // Ajuste a posição conforme necessário
}

function marcaPonto() {
  // Marca ponto quando a lata de cerveja atinge a borda final
  if (xBolinha + raioBolinha > width) {
    meusPontos += 1;
    somPonto.play();
    reiniciaBolinha();
  } else if (xBolinha - raioBolinha < 0) {
    pontosDoOponente += 1;
    somPonto.play();
    reiniciaBolinha();
  }
}

function reiniciaBolinha() {
  xBolinha = width / 2;
  yBolinha = height / 2;
  velocidadeXBolinha *= -1;
}

function calculaChanceDeErrar() {
  if (pontosDoOponente >= meusPontos) {
    chanceDeErrar += 3;  // Aumentei o incremento
    if (chanceDeErrar >= 10) {
      chanceDeErrar = 90;  // Ajustei o limite superior
    }
  } else {
    chanceDeErrar -= 3;  // Aumentei o decremento
    if (chanceDeErrar <= 6) {
      chanceDeErrar = 80;  // Ajustei o limite inferior
    }
  }
}

function mousePressed() {
  // Atualiza a posição da raquete com base na posição do clique
  toqueY = mouseY - raqueteRaio; // Ajuste para centralizar a raquete na posição do clique
  yRaquete = constrain(toqueY, 5, height - raqueteDiametro - 5); // Garante que a raquete fique dentro dos limites
}
