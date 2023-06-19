<template>
    <div id="maze-frame">
      <div class="point" 
           v-for="point in points"
           :key="point.x + ',' + point.y"
           :style="{top: point.y + 'px', left: point.x + 'px'}"></div>
      <div class="rover" 
           :style="{top: rover.y + 'px', left: rover.x + 'px'}">X</div>
      <div>
            <canvas id="myChart"></canvas></div>
    </div>
</template>

  
<script>
import axios from 'axios';
//import { Chart } from 'chart.js';


export default {
    data() {
        return {
            points: [],
            rover: {x: 0, y: 0}  // Add this line
        }
     },
   methods: {

    fetchData() {
      axios.get('http://18.168.204.27:3001/pointQuery')
        .then(response => {

            // Sort the data by timestamp in ascending order
            const sortedData = response.data.sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));
            this.startPoint = new Date();

            console.log("start point: ", this.startPoint);

            const batchData = []; // Reset batchData for each batch

            console.log("batchData supposed to be empty here: ", batchData);
            console.log("sortedData:", sortedData);

            for (const point of sortedData) {
                const pointTime = new Date(point.timestamp);
                
                // If this point is within 5 seconds of the start point, add it to the batch
                if ((this.startPoint - pointTime) <= 10000 && (this.startPoint-pointTime) >= 0) {

                    batchData.push(point);

                } else if ((this.startPoint - pointTime) > 10000) {
                    // If this point is more than 5 seconds from the start point, stop processing
                    break;
                }
            }
            
            // Separate the data based on the 'BeaconType' property
            const redData = this.getLatestEntries(batchData.filter(point => point.BeaconType.startsWith('RED') && point.xcordinate <= 640 && point.ycordinate <= 480));
            const yellowData = this.getLatestEntries(batchData.filter(point => point.BeaconType.startsWith('YELLOW') && point.xcordinate <= 640 && point.ycordinate <= 480));
            const blueData = this.getLatestEntries(batchData.filter(point => point.BeaconType.startsWith('BLUE') && point.xcordinate <= 640 && point.ycordinate <= 480));

            console.log('redData:', redData);
            console.log('yellowData:', yellowData);
            console.log('blueData:', blueData);

            // Calculate the center points of each color group
            const redCenter = this.averageCoordinates(redData);
            const yellowCenter = this.averageCoordinates(yellowData);
            const blueCenter = this.averageCoordinates(blueData);

            console.log('redCenter:', redCenter);
            console.log('yellowCenter:', yellowCenter);
            console.log('blueCenter:', blueCenter);

            // Calculate the rover's coordinates
            const rover = {
                x: (redCenter.x + yellowCenter.x + blueCenter.x) / 3,
                y: (redCenter.y + yellowCenter.y + blueCenter.y) / 3,
            };

            // Update the rover's coordinates in the data
            this.rover = rover;

            // Filter the data based on the 'BeaconType' property
            const filteredData = batchData.filter(point => point.BeaconType == 'WL' && point.xcordinate <= 640 && point.ycordinate <= 480);

            // Transform the filtered data into the format you need
            this.points = filteredData.map(point => ({
                x: point.xcordinate,
                y: point.ycordinate
            }));

            console.log('batchData:', batchData);

            })
            .catch(error => {
            console.error(error);
            });
    },

    getLatestEntries(data) {
        // Sort the data by id in descending order
        const sortedData = data.sort((a, b) => b.id - a.id);
        
        // Get the entries for 'TL' and 'BR'
        const tlEntry = sortedData.find(point => point.BeaconType.endsWith('TL'));
        const brEntry = sortedData.find(point => point.BeaconType.endsWith('BR'));

        // Return the entries as an array (or return an empty array if the entries are not found)
        return [tlEntry, brEntry].filter(entry => entry !== undefined);
    },

    averageCoordinates(data) {
    // Check if data[0] and data[1] are defined before trying to access their properties
    if (data[0] && data[1]) {
        // Average the coordinates
        return {x: (data[0].xcordinate + data[1].xcordinate) / 2, y: (data[0].ycordinate + data[1].ycordinate) / 2};
    } else {
        // If data[0] or data[1] is undefined, return a default value
        return {x: 0, y: 0};
    }
},

    startPolling() {
      setInterval(this.fetchData, 1000);  // Fetch data every 5 seconds
    }
},

mounted() {
    this.startPolling();
}

// methods: {
//   getCoordinates(coordString) {
//     const points = [];
      
//     // Split the coordinate string into xxxx,yyyy pairs
//     for (let i = 0; i < coordString.length; i += 8) {
//       let pair = coordString.slice(i, i + 8);

//       // Split the pair into x and y components
//       let x = parseInt(pair.slice(0, 4), 16);
//       let y = parseInt(pair.slice(4, 8), 16);

//       // Zero out the 5 most significant bits
//       x = x & 0x07FF;
//       y = y & 0x07FF;

//       points.push({x: x, y: y});
//     }

//     return points;
//   },

//   fetchData() {
//     // Get the coordinate string
//     const coordString = "";

//     // Remove the '|' characters and get the coordinates
//     const points = this.getCoordinates(coordString.replace(/\|/g, ''));

//     // Set the points in the data
//     this.points = points;
//   },
  

// },

// mounted() {
//   this.fetchData();
// }


}
</script>


<style scoped>
.point {
  position: absolute;
  width: 1px;
  height: 1px;
  background-color: #FFF; 
}
.rover {
  position: absolute;
  
}
</style>
