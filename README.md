# ROS 2 Chatbot Control Script

Este script em Python é projetado para controlar um robô utilizando o sistema operacional de robô ROS 2 (Robot Operating System 2) e NAV2 (Navigation2). O script é capaz de interpretar comandos de texto fornecidos pelo usuário e navegar o robô para pontos de interesse.

## Introdução

O script interpreta comandos de texto fornecidos pelo usuário e utiliza expressões regulares para reconhecer padrões relacionados a pontos de interesse, posições iniciais e comandos de finalização. Com base nessas entradas, o robô é direcionado para navegar até os destinos desejados.

## Dependências

Antes de executar o script, certifique-se de ter as seguintes dependências instaladas:

- `rclpy`: Biblioteca cliente do ROS 2 para Python.
- `nav2_simple_commander`: Pacote para controle de navegação.
- `geometry_msgs.msg`: Tipos de mensagem para geometria em ROS 2.
- `tf_transformations`: Biblioteca para manipulação de transformações de coordenadas.


# Documentação do Código

## Dicionário de Comandos `my_dict`

O dicionário `my_dict` mapeia palavras-chave para coordenadas tridimensionais no espaço. Cada chave representa um comando, e o valor associado é a correspondente coordenada.

Exemplo:

```python
my_dict = {
    'fim de turno': 'sair',
    'sair': "sair",
    'finalizar': "sair",
    'inicio': "0.0, 0.0, 0.0",
    'chave de fenda': "1.03, 0.0, 0.0",
    'puro malte': "3.61, 0.04, 0.0",
    'correia': "2.56, 2.34, 0.0",
    'embalagem': "1.74, 1.08, 0.0",
    'ponto 1': "0.33, 2.0, 0.0",
}
```

## Função `translate`

A função `translate` converte comandos de texto em coordenadas tridimensionais. Ela utiliza expressões regulares para identificar comandos específicos no texto e retorna as coordenadas correspondentes.


```python
def translate(text):
    nav = BasicNavigator()
    point = re.compile(r'\b(?:ponto\s?(\d+?)|chave\s?de\s?fenda|puro\s?malte|correia|embalagem)\b', re.IGNORECASE)
    zero_pose = re.compile(r'\b(?:inicio|inicial)\b', re.IGNORECASE)
    sair = re.compile(r'\b(?:sair|\s?finalizar|fim\s?de\s?turno)\b', re.IGNORECASE)
```


## Construção do pacote

1. Abra um terminal e navegue até o diretório raiz do seu espaço de trabalho:

   ```bash
   cd meu_ros
   ```
2. Execute o comando `colcon build` para compilar o projeto:

   ```bash
   colcon build
   ```
3. Após a compilação, execute o comando `source` para adicionar o pacote ao seu ambiente:

   ```bash
   # Use setup.zsh caso esteja utilizando o ZSH
   source install/setup.bash
   ```
4. Para executar o projeto, execute o comando `ros2 run` + nome do pacote + nome do nó:

   ```bash
   ros2 run meu_ros chatbot
   ```

## Execução do Projeto - Video de Funcionamento

link: https://drive.google.com/file/d/12GtYLiw3lmVfDMYTkkp9lANDR0bmbWpj/view?usp=sharing
