var noble = require('noble');


noble.on('stateChange', function(state) {
  if (state === 'poweredOn') {
    noble.startScanning();
  } else {
    noble.stopScanning();
  }
});

var data1 = null, data2 = null;

var log = [];
var fs = require('fs');

function on_sensor_data(data, is_notification)
{
    if (is_notification)
    {
        // dont care
    }
    //console.log('data: ', data);
    //console.log('len: ', data.length);
    if (!data1 || !data2)
    {
        if (data.length == 20)
        {
            data1 = data;
        }
        else
        {
            data2 = data;
        }
    }
    else
    {
        buf = Buffer.concat([data1, data2], data1.length + data2.length);
        data1 = null;
        data2 = null;

        var sens_data = {
            accel: {
                x: buf.readInt16BE(0),
                y: buf.readInt16BE(2),
                z: buf.readInt16BE(4)
            },
            gyro: {
                x: buf.readInt16BE(8),
                y: buf.readInt16BE(10),
                z: buf.readInt16BE(12)
            },
            magno: {
                x: buf.readInt16LE(14),
                y: buf.readInt16LE(16),
                z: buf.readInt16LE(18)
            },
            temperature: buf.readInt16BE(6),
            timestamp: buf.readInt32LE(21),
            id: buf.readUInt8(25)

        };
        console.log(sens_data);
        log.push(sens_data);
        if (log.length == 3000)
        {
            console.log('logging last 10s');
            fs.writeFile("testvector_circles", JSON.stringify(log), function(err) {
                if(err) {
                    return console.log(err);
                }

                console.log("The file was saved!");
                log = [];
            }); 
        }
    }

};


noble.on('discover', function(peripheral) {


    if (peripheral.id == 'c90a5ad6b7fa')
    {

        console.log('found it!');


        //console.log('peripheral discovered (' + peripheral.id +
            //' with address <' + peripheral.address +  ', ' + peripheral.addressType + '>,' +
                //' connectable ' + peripheral.connectable + ',' +
                //' RSSI ' + peripheral.rssi + ':');
        //console.log('\thello my local name is:');
        //console.log('\t\t' + peripheral.advertisement.localName);
        //console.log('\tcan I interest you in any of the following advertised services:');
        //console.log('\t\t' + JSON.stringify(peripheral.advertisement.serviceUuids));

        var serviceData = peripheral.advertisement.serviceData;
        if (serviceData && serviceData.length) {
            console.log('\there is my service data:');
            for (var i in serviceData) {
                console.log('\t\t' + JSON.stringify(serviceData[i].uuid) + ': ' + JSON.stringify(serviceData[i].data.toString('hex')));
            }
        }
        if (peripheral.advertisement.manufacturerData) {
            console.log('\there is my manufacturer data:');
            console.log('\t\t' + JSON.stringify(peripheral.advertisement.manufacturerData.toString('hex')));
        }
        if (peripheral.advertisement.txPowerLevel !== undefined) {
            console.log('\tmy TX power level is:');
            console.log('\t\t' + peripheral.advertisement.txPowerLevel);
        }



        peripheral.connect(function(err){
            if (err)
            {
                console.log('error:',err);
            }

            peripheral.discoverAllServicesAndCharacteristics(function(error, services, characteristics){
                if (err)
                {
                    console.log('serv/desc error:',err);
                }


                console.log('services:');
                console.log(services);
                console.log('characteristics:');
                console.log(characteristics);
                var c;

                for (i in characteristics)
                {
                    c = characteristics[i];
                    if (c.uuid =='0000babe1212efde1523785fef13d123')
                    {
                        console.log('found the sensor characteristic');
                        break;
                    }
                }
                if (c.uuid != '0000babe1212efde1523785fef13d123')
                {
                    console.log('did NOT find the sensor characteristic');
                    return 1;
                }
                c.on('data', on_sensor_data);
                c.subscribe(function(err){
                    if (err)
                    {
                        console.log('error reading char',err);
                        return;
                    }
                });

            });

        });
    }

});

