
# Projeto Robô Autônomo - LARIS

Este repositório contém os arquivos e configurações para o desenvolvimento do **Robô Autônomo** do **Laboratório de Robótica e Sistemas Inteligentes (LARIS)**.

O projeto utiliza o **ROS 2 Humble Hawksbill** como framework de desenvolvimento robótico.
Desenvolvido por:

- Robson Rogério (version 1.0 ROS1 noetic)
- Augusto Amaral (version 2.0 conversão ROS1 para ROS2 docker: isaac_ros_common dist. humble)
- Paulo Morais (version 2.1) Humble-zlac

Projeto MAI/DAI Robovisor 2023 sob supervisão do Prof. Dr. Roberto Inoue

---

## Índice

- [Instalação do ROS 2 Humble](#instalação-do-ros-2-humble)
- [Criando o Workspace](#criando-o-workspace)
- [Criando um Pacote](#criando-um-pacote)
- [Configuração do CMakeLists.txt](#configuração-do-cmakeliststxt)
- [Testando o URDF](#testando-o-urdf)
  - [1️⃣ Testar o Xacro separadamente](#1️⃣-testar-o-xacro-separadamente)
- [Dependências adicionais](#dependências-adicionais)
- [Recomendações de Versionamento (Git)](#recomendações-de-versionamento-git)

---

## Instalação do ROS 2 Humble

> Caso você ainda não tenha o ROS 2 Humble instalado, siga os passos abaixo (para Ubuntu 22.04):

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

Adicione ao seu `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Criando o Workspace

A melhor prática é criar um novo diretório de workspace para cada projeto ROS 2.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Depois de adicionar pacotes no workspace, sempre rode:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
Para verificar as dependências do pacote utilize o comando: 
```
rosdep install --from-paths src --ignore-src -r -y
```
---
## Criando um Pacote

Certifique-se de estar dentro da pasta `/src` do workspace antes de criar o pacote.

```bash
cd ~/ros2_ws/src
```

Para criar um pacote usando o CMake como sistema de build:

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <nome_do_pacote>
```

Exemplo:

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 laris_robot
```

---

## Configuração do CMakeLists.txt

Se o seu pacote contém arquivos de launch, lembre-se de adicionar no `CMakeLists.txt`:

```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

Isso garante que os arquivos dentro da pasta `launch/` sejam instalados corretamente.

---

## Testando o URDF

### Instale as dependências para visualização no RViz2:

```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2
```

---

### 1️⃣ Testar o Xacro separadamente

Antes de rodar o launch, é recomendado validar o Xacro:

```bash
xacro ~/ros2_ws/src/laris_robot/urdf/robot.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf
```

Isso permite verificar erros de sintaxe ou estrutura antes de abrir no RViz.

---

## Dependências adicionais

- **xacro** (caso não venha instalado com o desktop, use: `sudo apt install ros-humble-xacro`)
- Outros pacotes customizados do LARIS (presentes neste repositório ou em repositórios internos)

---

## Recomendações de Versionamento (Git)

Para manter a organização e facilitar o trabalho colaborativo, siga este padrão e boas práticas ao utilizar o Git neste projeto.

### 📌 Padrão de Nome para Branches

O nome da sua branch deve seguir o formato:

```
<distro-do-ros>-<nome-da-funcionalidade(se simulado)-ou-driver>
```

**Exemplos de branches válidas:**

- `humble-pico`
- `humble-sim-urdf`
- `humble-zlac`

---

### 📌 Criando uma nova branch

Antes de começar qualquer desenvolvimento:

```bash
git checkout main
git pull origin main
git checkout -b humble-nome-da-funcionalidade
```

Exemplo real:

```bash
git checkout -b humble-pico
```

---

### 📌 Mantendo sua branch atualizada com a `main`

Durante o desenvolvimento, atualize sua branch regularmente para incorporar as mudanças mais recentes da `main`:

```bash
git checkout main
git pull origin main
git checkout <nome-branch>
git merge main
```

Se houver conflitos, resolva-os manualmente e depois:

```bash
git add .
git commit -m "LIDAR-implemantation"
```

---

### 📌 Finalizando o desenvolvimento

Quando terminar sua funcionalidade:

```bash
git push origin <nome-branch>
```


---

## Observações finais

Este projeto faz parte do desenvolvimento de um sistema de robô móvel autônomo para aplicações de pesquisa do **Laboratório de Robótica e Sistemas Inteligentes (LARIS)**.

Qualquer dúvida, abra uma issue ou entre em contato com o time de desenvolvimento.

---
