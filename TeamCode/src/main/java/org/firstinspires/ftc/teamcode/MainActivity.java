package org.firstinspires.ftc.teamcode;

import android.app.AlertDialog;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;

import com.google.zxing.Result;

import me.dm7.barcodescanner.zxing.ZXingScannerView;

public class MainActivity extends AppCompatActivity implements ZXingScannerView.ResultHandler {
    private ZXingScannerView mScannerView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

    }

    public void QrScanner(View view){


        mScannerView = new ZXingScannerView(this);   // Programmatically initialize the scanner view
        setContentView(mScannerView);

        mScannerView.setResultHandler(this); // Register ourselves as a handler for scan results.
        mScannerView.startCamera();         // Start camera

    }

    @Override
    public void onPause() {
        super.onPause();
        mScannerView.stopCamera();           // Stop camera on pause
    }

    @Override
    public void handleResult(Result rawResult) {
        // Do something with the result here

        Log.e("handler", rawResult.getText()); // Prints scan results
        Log.e("handler", rawResult.getBarcodeFormat().toString()); // Prints the scan format (qrcode)
        final String QRCode =  rawResult.getText();

        // show the scanner result into dialog box.
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Scan Result");
        String allianceColor;
        double waitTime;
        String startingPosition;
        if (QRCode.substring(0,1).equals ("r")){
            allianceColor = "red";
        } else {
            allianceColor = "blue";
        }
        waitTime = Integer.parseInt(QRCode.substring(1,3));
        if (QRCode.substring(3,4).equals("r")){
            startingPosition = "right";
        } else {
            startingPosition = "left";
        }
        Double.toString(waitTime);

        builder.setMessage("You are on the " + allianceColor + " alliance, with a starting posistion on the " + startingPosition + ". There will be " + waitTime + " seconds before your autonomous begins.");
        AlertDialog alert1 = builder.create();
        alert1.show();

        // If you would like to resume scanning, call this method below:
       mScannerView.resumeCameraPreview(this);
    }
}
