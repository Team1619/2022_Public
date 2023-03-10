//js code to run the graph page of the webdashboard,
//which graphs data from the robot code using the values page

//Array of color strings from style.css used sequentially for each dataset put on the graph
const colors = ['--red', '--green', '--blue', '--purple', '--yellow', '--pink', '--orange'];

//Array of colored arrows used sequentially for each dataset put on the graph
function mkArrow(color) {
    let canvas = document.createElement("canvas");
    canvas.id = "headingarrow-" + color;
    canvas.width=20
    canvas.height=20
    let ctx = canvas.getContext("2d");
    ctx.fillStyle = color;
    ctx.textAlign = "center"
    ctx.font = "15px Arial";
    ctx.fillText("➤", canvas.width/2, canvas.height/2);
    return canvas
}
const arrows = [mkArrow('red'), mkArrow('green'), mkArrow('blue'), mkArrow('purple'), mkArrow('yellow'), mkArrow('pink'), mkArrow('orange')];

//Code used to remove a dataset from the graph on double click of the legend entry
//Overrides the original legend onClick
//Needed because chart.js doesn't support native double click events

//Keeps track of the last time a legend click event occurred
//If a double click on the body occurs rapidly afterward the event can be connected to the legend
let lastLegendItemClickTime = 0;

//The last legend element that was clicked
let lastLegendItem;

//If the graph is currently paused
let paused = false;

//Keeps track of the original legend onClick so that it can still be called
//The original onClick toggles if the dataset is show on the graph
const originalLegendOnClick = Chart.defaults.global.legend.onClick;

//Overwrites the default legend onClick event
Chart.defaults.global.legend.onClick = function (e, legendItem) {
    //Records the time when the click occurs
    lastLegendItemClickTime = Date.now();

    //Tracks the element on which the click occurred
    lastLegendItem = legendItem;

    //Calls the original legend onClick event allowing for the default show/hide dataset behavior to be maintained
    originalLegendOnClick.call(this, e, legendItem);
};

//Gets the chart canvas to be used for the graph from the html page
let graphCanvas = document.getElementById("graph");

//Create the graph using chart.js
let graph = new Chart(graphCanvas, {
    //Configures graph type to scatter plot
    type: 'scatter',
    data: {
        //Sets the datasets to an empty array to be filled later
        datasets: []
    },
    options: {
        //Configures graph to give hover text over points
        responsive: true,
        //Configures graph to allow for access ratio changes
        maintainAspectRatio: false,
        legend: {
            labels: {
                //Set the graph legend text to the --white color specified in style.css
                fontColor: getComputedStyle(document.documentElement).getPropertyValue('--white'),
                //Set the font size of the labels in the legend to 18px
                fontSize: 18
            }
        },
        scales: {
            yAxes: [{
                ticks: {
                    //Sets the graph yAxes tick color to the --graph color specified in style.css
                    fontColor: getComputedStyle(document.documentElement).getPropertyValue('--graph')
                }
            }],
            xAxes: [{
                ticks: {
                    //Sets the graph xAxes tick color to the --graph color specified in style.css
                    fontColor: getComputedStyle(document.documentElement).getPropertyValue('--graph')
                }
            }]
        },
        animation: {
            //Removes graph dataset change animations
            duration: 0
        },
        plugins: {
            zoom: {
                pan: {
                    enabled: true,
                    mode: function({ chart }) {
                        let mode = "";
                        if($("#scroll")[0].checked) {
                            if(!$("#autorangeX")[0].checked) {
                                mode += "x";
                            }
                            if(!$("#autorangeY")[0].checked) {
                                mode += "y";
                            }
                        }
                        return mode;
                    },
                },
                zoom: {
                    enabled: true,
                    mode: function({ chart }) {
                        let mode = "";
                        if($("#zoom")[0].checked) {
                            if(!$("#autorangeX")[0].checked) {
                                mode += "x";
                            }
                            if(!$("#autorangeY")[0].checked) {
                                mode += "y";
                            }
                        }
                        return mode;
                    },
                    speed: 0.1
                }
            }
        }
    }
});

//Create context menu
let BB = graphCanvas.getBoundingClientRect();
let offsetX = BB.left;
let offsetY = BB.top;

graphCanvas.oncontextmenu = function (e) {
    e.preventDefault();
    e.stopPropagation();

    let graphContextMenu = $("#graphContextMenu");

    let x = e.clientX + offsetX;
    let y = e.clientY + offsetY;

    graphContextMenu.css({left: x, top: y});
    graphContextMenu.show();

    return false;
};

graphCanvas.onclick = function (e) {
    e.preventDefault();
    e.stopPropagation();

    let graphContextMenu = $("#graphContextMenu");

    graphContextMenu.hide();
};

$("#graphContextMenu").onclick = function (e) {
    e.preventDefault();
    e.stopPropagation();
};

$("#autorangeX").change(function (e) {
    if($("#autorangeX")[0].checked) {
        graph.resetZoom();
    }
});

$("#autorangeY").change(function (e) {
    if($("#autorangeY")[0].checked) {
        graph.resetZoom();
    }
});

//Called when a double click occurs on the page body
function doubleClick() {
    if (Date.now() - lastLegendItemClickTime < 500) {
        //If a click on the legend occurred less than 500 milliseconds ago trigger legend event

        //Get the graph values put into sessionStorage by the values pages
        let graphValues = JSON.parse(sessionStorage['graph_values']);

        //Set the dataset on which the double click occurred to undefined to tell the values page it has been removed
        graphValues[lastLegendItem.text] = undefined;

        //Write the values new data into sessionStorage
        sessionStorage['graph_values'] = JSON.stringify(graphValues);

        setTimeout(function () {
            //Wait 50 milliseconds to ensure sessionStorage update complete the update the graph

            //Remove the dataset on which the double click occurred from the graph
            graph.data.datasets.splice(lastLegendItem.datasetIndex, 1);

            //Update the graph to show changes
            graph.update();
        }, 50);
    } else {
        //If doubleClick not related to the legend open menu
        setVisibility(document.getElementById("menuPopup"), true);
    }
}

//Closes the menu, called when the close button is pressed
function closeMenu() {

    //Hide the menu
    setVisibility(document.getElementById("menuPopup"), false);
}

//Called periodically to add another data point to each dataset on the graph
//Uses data send to the values page of the webdashboard
function update() {
    if(paused) {
        return;
    }

    //Get the graph values put into sessionStorage by the values pages
    let graphValues = JSON.parse(sessionStorage['graph_values']);

    //Loop through each dataset in graphValues
    for (let dataset of Object.getOwnPropertyNames(graphValues)) {

        //If the dataset has been updated then add it to the graph
        if (graphValues[dataset].updated) {
            if (dataset.startsWith("gr_")) {
                //If the value is a full graph dataset then add the dataset to the graph as is

                //Adding the dataset to the graph with its name and values
                addData(dataset, graphValues[dataset].value);

                //Clear dataset from sessionStorage
                //Prevents the dataset from being updated again until the values page adds it again
                graphValues[dataset] = undefined;
            } else {
                if ((typeof graphValues[dataset].value) === "string" && graphValues[dataset].value.includes(",")) {
                    //If the dataset includes point with x and y values then add a point with those values

                    //Adding the dataset to the graph with its name creating x and v values
                    addData(dataset, [{

                        //Sets x value to the first value of the point
                        x: graphValues[dataset].value.split(",")[0],

                        //Sets y value to the second value of the point
                        y: graphValues[dataset].value.split(",")[1]
                    }]);

                    graphValues[dataset].updated = false;
                } else {
                    //If the dataset only includes a y-value use the time for the x-value
                    //This allows the user to see how the variable changes over time

                    //Adding the dataset to the graph with its name creating x and v values
                    addData(dataset, [{

                        //Set the x value to the current time, using modulo to prevent the value from getting to huge
                        x: new Date().getTime() % 1000000,

                        //Set the y to the data value
                        y: graphValues[dataset].value
                    }]);

                    graphValues[dataset].updated = false;
                }
            }
        }
    }

    sessionStorage['graph_values'] = JSON.stringify(graphValues);
}

//Adds data to a dataset, creating a new dataset if needed
//"label" is the label of the dataset
//"data" is the data to be added to the dataset
function addData(label, data) {

    //Loop through all of the datasets on the graph to try and find a dataset with a matching label
    for (let dataset of graph.data.datasets) {

        if (dataset.label === label) {
            //If the labels match add the new data to the dataset

            //Loop through each point and add it to the dataset
            for (let point of data) {
                dataset.data.push(point);
            }

            //Update the graph to show changes
            graph.update();
            return;
        }
    }

    //If no dataset is found matching the label create a new dataset

    //Pick the color for the dataset based upon the current number of datasets
    //This ensures each dataset will have a different color from the colors array
    //The colors will wrap around if there are 7 or more datasets
    let color = colors[graph.data.datasets.length % 7];
    let arrow = arrows[graph.data.datasets.length % 7]

    //Create the dataset
    let dataset = {

        //Set the dataset label the the label passed in
        label: label,

        //Sets the label to the --foreground color specified in style.css
        labelColor: getComputedStyle(document.documentElement).getPropertyValue('--foreground'),

        //Sets the point border and background colors to the color picked from the colors array
        borderColor: getComputedStyle(document.documentElement).getPropertyValue(color),
        backgroundColor: getComputedStyle(document.documentElement).getPropertyValue(color),

        //Sets the radius of the points to 5px
        pointRadius: 5,

        pointStyle: arrow,
        pointRotation: data.map(pose => pose.h),

        //Sets the data in the dataset to the data passed in
        data: data
    };

    //Add the dataset to the graph
    graph.data.datasets.push(dataset);

    //Update the graph to show changes
    graph.update();
}

//Removes all datasets from the graph
function removeData() {
    //Clears all datasets
    graph.data.datasets = [];

    //Updates the chart to display to changes
    graph.update();
}

// Pauses and resumes graph
function toggleGraphPause() {
    paused = !paused;

    $("#pauseButton").text(paused ? "Resume" : "Pause")
}

//Used to show and hide an element
//"element" is the element
//"visible" is whether the element should be displayed
function setVisibility(element, visible) {
    //Show or hide the element using the visibility property based on the visible variable
    if (visible) {
        element.style.visibility = "visible";
    } else {
        element.style.visibility = "hidden";
    }

    //Get children of the element
    let children = element.children;

    //Loop through and show or hide the element's children using the visibility property based on the visible variable
    for (let c = 0; c < children.length; c++) {
        if (visible) {
            children[c].style.visibility = "visible";
        } else {
            children[c].style.visibility = "hidden";
        }
    }
}

//Updates each dataset on the graph every 16 milliseconds
//Similar to the speed of robot updates to avoid missing data while not using too much processing power
setInterval(update, 15);

// custom_helper_canvas_drawPoint from https://jsfiddle.net/nyammy/vx8cd9w4/3/
var PI = Math.PI;
var RAD_PER_DEG = PI / 180;
var DOUBLE_PI = PI * 2;
var HALF_PI = PI / 2;
var QUARTER_PI = PI / 4;
var TWO_THIRDS_PI = PI * 2 / 3;
var custom_helper_canvas_drawPoint = function(ctx, style, radius, x, y, rawRotation) {
    var rotation = rawRotation * -1.0
    var type, xOffset, yOffset, size, cornerRadius;
    var rad = (rotation || 0) * RAD_PER_DEG;

    if (style && typeof style === 'object') {
        type = style.toString();
        if (type === '[object HTMLImageElement]' || type === '[object HTMLCanvasElement]') {
            // Added from here
            ctx.save();
            ctx.translate(x, y);
            ctx.rotate(rotation * Math.PI / 180);
            ctx.drawImage(style, -1* style.width / 2, -1* style.height / 2, style.width, style.height);
            // to here
            //ctx.drawImage(style, x - style.width / 2, y - style.height / 2, style.width, style.height);
            //Added from here
            ctx.restore();
            // to here
            return;
        }
    }

    if (isNaN(radius) || radius <= 0) {
        return;
    }

    ctx.beginPath();

    switch (style) {
        // Default includes circle
        default:
            ctx.arc(x, y, radius, 0, DOUBLE_PI);
            ctx.closePath();
            break;
        case 'triangle':
            ctx.moveTo(x + Math.sin(rad) * radius, y - Math.cos(rad) * radius);
            rad += TWO_THIRDS_PI;
            ctx.lineTo(x + Math.sin(rad) * radius, y - Math.cos(rad) * radius);
            rad += TWO_THIRDS_PI;
            ctx.lineTo(x + Math.sin(rad) * radius, y - Math.cos(rad) * radius);
            ctx.closePath();
            break;
        case 'rectRounded':
            // NOTE: the rounded rect implementation changed to use `arc` instead of
            // `quadraticCurveTo` since it generates better results when rect is
            // almost a circle. 0.516 (instead of 0.5) produces results with visually
            // closer proportion to the previous impl and it is inscribed in the
            // circle with `radius`. For more details, see the following PRs:
            // https://github.com/chartjs/Chart.js/issues/5597
            // https://github.com/chartjs/Chart.js/issues/5858
            cornerRadius = radius * 0.516;
            size = radius - cornerRadius;
            xOffset = Math.cos(rad + QUARTER_PI) * size;
            yOffset = Math.sin(rad + QUARTER_PI) * size;
            ctx.arc(x - xOffset, y - yOffset, cornerRadius, rad - PI, rad - HALF_PI);
            ctx.arc(x + yOffset, y - xOffset, cornerRadius, rad - HALF_PI, rad);
            ctx.arc(x + xOffset, y + yOffset, cornerRadius, rad, rad + HALF_PI);
            ctx.arc(x - yOffset, y + xOffset, cornerRadius, rad + HALF_PI, rad + PI);
            ctx.closePath();
            break;
        case 'rect':
            if (!rotation) {
                size = Math.SQRT1_2 * radius;
                ctx.rect(x - size, y - size, 2 * size, 2 * size);
                break;
            }
            rad += QUARTER_PI;
        /* falls through */
        case 'rectRot':
            xOffset = Math.cos(rad) * radius;
            yOffset = Math.sin(rad) * radius;
            ctx.moveTo(x - xOffset, y - yOffset);
            ctx.lineTo(x + yOffset, y - xOffset);
            ctx.lineTo(x + xOffset, y + yOffset);
            ctx.lineTo(x - yOffset, y + xOffset);
            ctx.closePath();
            break;
        case 'crossRot':
            rad += QUARTER_PI;
        /* falls through */
        case 'cross':
            xOffset = Math.cos(rad) * radius;
            yOffset = Math.sin(rad) * radius;
            ctx.moveTo(x - xOffset, y - yOffset);
            ctx.lineTo(x + xOffset, y + yOffset);
            ctx.moveTo(x + yOffset, y - xOffset);
            ctx.lineTo(x - yOffset, y + xOffset);
            break;
        case 'star':
            xOffset = Math.cos(rad) * radius;
            yOffset = Math.sin(rad) * radius;
            ctx.moveTo(x - xOffset, y - yOffset);
            ctx.lineTo(x + xOffset, y + yOffset);
            ctx.moveTo(x + yOffset, y - xOffset);
            ctx.lineTo(x - yOffset, y + xOffset);
            rad += QUARTER_PI;
            xOffset = Math.cos(rad) * radius;
            yOffset = Math.sin(rad) * radius;
            ctx.moveTo(x - xOffset, y - yOffset);
            ctx.lineTo(x + xOffset, y + yOffset);
            ctx.moveTo(x + yOffset, y - xOffset);
            ctx.lineTo(x - yOffset, y + xOffset);
            break;
        case 'line':
            xOffset = Math.cos(rad) * radius;
            yOffset = Math.sin(rad) * radius;
            ctx.moveTo(x - xOffset, y - yOffset);
            ctx.lineTo(x + xOffset, y + yOffset);
            break;
        case 'dash':
            ctx.moveTo(x, y);
            ctx.lineTo(x + Math.cos(rad) * radius, y + Math.sin(rad) * radius);
            break;
    }

    ctx.fill();
    ctx.stroke();
};

Chart.helpers.canvas.drawPoint = custom_helper_canvas_drawPoint;