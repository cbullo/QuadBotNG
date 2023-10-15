var i = 0;

var objects = new Array(4 * 6);

function attachLegLog(es, tempCanvas, angleCanvas, legName) {
    
    // let es_tempo = new EventSource(`/sse/tempo/${legName}`);
    // let es_tempz = new EventSource(`/sse/tempz/${legName}`);
  
    // let es_theta = new EventSource(`/sse/theta/${legName}`);
    // let es_gamma = new EventSource(`/sse/gamma/${legName}`);
    // let es_z     = new EventSource(`/sse/z/${legName}`);
     
    // objects[6 * i+0] = es_tempi;
    // objects[6 * i+1] = es_tempo;
    // objects[6 * i+2] = es_tempz;
    // objects[6 * i+3] = es_theta;
    // objects[6 * i+4] = es_gamma;
    // objects[6 * i+5] = es_z;
    // i++;

    let chart_temp = new Chart(tempCanvas, {
        type: 'line',
        data: {
          datasets: [{ 
              data: [{x: 0, y: 10}, {x: 1, y: 20}, {x: 2, y: 30}],
              label: "I",
              borderColor: "#3e95cd",
              fill: false
            }, { 
              data: [{x: 0, y: 12}, {x: 1, y: 22}, {x: 2, y: 32}],
              label: "O",
              borderColor: "#8e5ea2",
              fill: false
            }, { 
              data: [{x: 0, y: 14}, {x: 1, y: 24}, {x: 2, y: 34}],
              label: "Z",
              borderColor: "#3cba9f",
              fill: false
            }
          ]
        },
        options: {
          plugins: {
            title: {
              display: true,
              text: `Temperature ${legName}`
            },
            datalabels: {
              display: function(context) {
                  return context.dataIndex === context.dataset.data.length - 1;
              },
              formatter: function(value, context) {
                return Math.round(value.y)
              },
              color: 'red'
            }
          },
          scales: {
            x: {
              type: 'linear'
            },
            y: {
              min: 0,
              max: 80,
              position: 'right'
            }
          },
          elements: {
            point: {
              pointStyle: false
            }
          },
          responsive: true,
          maintainAspectRatio: true,
          aspectRatio: 3
        }
      });
  
        let chart_angle = new Chart(angleCanvas, {
          type: 'line',
          data: {
            datasets: [{ 
                data: [{x: 0, y: 10}, {x: 1, y: 20}, {x: 2, y: 30}],
                label: "Theta",
                borderColor: "#3e95cd",
                fill: false
              }, { 
                data: [{x: 0, y: 12}, {x: 1, y: 22}, {x: 2, y: 32}],
                label: "Gamma",
                borderColor: "#8e5ea2",
                fill: false
              }, { 
                data: [{x: 0, y: 14}, {x: 1, y: 24}, {x: 2, y: 34}],
                label: "Z",
                borderColor: "#3cba9f",
                fill: false
              }
            ]
          },
          options: {
            plugins: {
              title: {
                display: true,
                text: `Angle ${legName}`
              },
              datalabels: {
                display: function(context) {
                    return context.dataIndex === context.dataset.data.length - 1;
                },
                formatter: function(value, context) {
                  return value.y;
                },
                color: 'red'
              }
            },
            scales: {
              x: {
                type: 'linear'
              },
              y: {
                min: -4,
                max: 4
              }
            },
            elements: {
              point: {
                pointStyle: false
              }
            },
            responsive: true,
            maintainAspectRatio: true,
            aspectRatio: 3
          }
        });
      
      function updateData(chart, data, timeStamp, value) {
        if (data.length > 100) {
          data.shift();
        }
        chart.options.scales.x.min = timeStamp - 0.2 * 100;
        chart.options.scales.x.max = timeStamp;
        data.push({x: timeStamp, y: value});
      }
  
      try {
        es.addEventListener("message", (event) => {
          let values = JSON.parse(event.data);
          updateData(chart_temp, chart_temp.data.datasets[0].data, values.timestamp, values[legName].temperature_i);
          updateData(chart_temp, chart_temp.data.datasets[1].data, values.timestamp, values[legName].temperature_o);
          updateData(chart_temp, chart_temp.data.datasets[2].data, values.timestamp, values[legName].temperature_z);
          updateData(chart_angle, chart_angle.data.datasets[0].data, values.timestamp, values[legName].theta);
          updateData(chart_angle, chart_angle.data.datasets[1].data, values.timestamp, values[legName].gamma);
          updateData(chart_angle, chart_angle.data.datasets[2].data, values.timestamp, values[legName].z);
  
          chart_temp.update('none');
          chart_angle.update('none');
        });

        // es_tempo.onmessage = function got_packet2(msg) {
        //   updateData(chart_temp.data.datasets[1].data, msg);
        //   chart_temp.update('none');
        // };

        // es_tempz.onmessage = function got_packet3(msg) {
        //   updateData(chart_temp.data.datasets[2].data, msg);
        //   chart_temp.update('none');
        // };
  
        // es_theta.onmessage = function got_packet4(msg) {
        //   updateData(chart_angle.data.datasets[0].data, msg);
        //   chart_angle.update('none');
        // };

        // es_gamma.onmessage = function got_packet5(msg) {
        //   updateData(chart_angle.data.datasets[1].data, msg);
        //   chart_angle.update('none');
        // };

        // es_z.onmessage = function got_packet6(msg) {
        //   updateData(chart_angle.data.datasets[2].data, msg);
        //   chart_angle.update('none');
        // };
  
        // setInterval(() => {
        //   updateData(chart_angle.data.datasets[0].data, {timeStamp: i++, data: 2});
        //   chart_angle.update('none');
        // }, 100);
  
        // setInterval(() => {
        //   updateData(chart_angle.data.datasets[1].data, {timeStamp: i++, data: 5});
        //   chart_angle.update('none');
        // }, 100);
    
        /* there is no onclose() for EventSource */
    
      } catch(exception) {
        alert("<p>Error" + exception);  
      }
    
    } 
document.addEventListener("DOMContentLoaded", () => {
  let es = new EventSource(`/sse`);
  Chart.register(ChartDataLabels);
  attachLegLog(es, document.getElementById("fl-temp"), document.getElementById("fl-angle"), "fl")
  attachLegLog(es, document.getElementById("fr-temp"), document.getElementById("fr-angle"), "fr")
  attachLegLog(es, document.getElementById("bl-temp"), document.getElementById("bl-angle"), "bl")
  attachLegLog(es, document.getElementById("br-temp"), document.getElementById("br-angle"), "br")
} , true);



