<template>
    <div class="buttons">
      <button id="go-button" :disabled="isMoving" @click="updateStatus('GO')">GO</button>
      <button id="stop-button" @click="updateStatus('STOP')">STOP</button>
    </div>
  </template>
  
  <script>
  export default {
    data() {
      return {
        isMoving: false,
      };
    },
    methods: {
      updateStatus(status) {
        if (status === 'GO') {
          this.isMoving = true;
        } else if (status === 'STOP') {
          this.isMoving = false;
        }
      },
    },
  };
  </script>
  
  <style scoped>
  /* Your styles here */
  </style>
  