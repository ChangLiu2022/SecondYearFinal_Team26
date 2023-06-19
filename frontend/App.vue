<template>
  <div id="app">
    <MazeTitle />
    <MazeFrame />
    <ControlButtons />
  </div>
</template>

<script>
import MazeTitle from './components/MazeTitle.vue';
import MazeFrame from './components/MazeFrame.vue';
import ControlButtons from './components/ControlButtons.vue';

export default {
  name: 'App',
  components: {
    MazeTitle,
    MazeFrame,
    ControlButtons,
  },
};
</script>

<style>
body {
    background-color: #444654;
    margin: 0;
    color: #F5F5F5;
    font-family: Arial, sans-serif;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    height: 100vh;
}

#app {
    display: flex;
    flex-direction: column;
    align-items: center;
    width: 70%;
    padding-top: 20px;
}

#maze-frame {
    width: 640px;
    height: 480px;
    position: relative;
    background-color: #000;
    margin-right: 30px;
}

/* .rover {
    position: absolute;
    top: 50px;
    left: 50px;
    color: #fff;
    font-size: 2rem;
} */
</style>
