package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.content.Context;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class ToggleUI extends SettingsActivity  {
    public boolean isNew = false;
    public Spinner spinner;
    public EditText editTxt;
    public Button toggleBtn;
    List<String> dataStr;
    String fileName;

    public void initSpinner(int id){
        spinner = (Spinner) findViewById(id);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, reverseStr(dataStr));
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinner.setAdapter(adapter);
    }

    public List<String> initData(String fileName){
        return readFile(fileName);
    }

    public String dataFromUser(int idSpinner, int idEditTxt){
        String newdata;
        if(!isNew) {
            spinner = (Spinner) findViewById(idSpinner);
            return spinner.getSelectedItem().toString();
        }
        else {
            editTxt = (EditText) findViewById(idEditTxt);
            newdata = editTxt.getText().toString();
            writeFile(fileName, dataStr, newdata);
            return newdata;
        }
    }

    public void toggleBtns(int idSpinner, int idBtn, int idEdt){
        spinner = (Spinner) findViewById(idSpinner);
        toggleBtn = (Button) findViewById(idBtn);
        editTxt = (EditText) findViewById(idEdt);
        if(!isNew) {
            isNew = true;
            spinner.setVisibility(View.GONE);
            toggleBtn.setText("Back to list");
            editTxt.setVisibility(View.VISIBLE);
        }
        else{
            isNew = false;
            spinner.setVisibility(View.VISIBLE);
            toggleBtn.setText("Set New IP");
            editTxt.setVisibility(View.GONE);
        }

    }

    public List<String> reverseStr(List<String> str){
        List<String> revStr = new ArrayList<String>();
        int index;
        //Reverse str so that the most recent IP is on top of spinner
        for(index = str.size() - 1; index >= 0; index--){
            revStr.add(str.get(index));
        }
        return revStr;
    }

    public List<String> readFile(String fileName){
        List<String> str = new ArrayList<String>();
        String line;

        try {
            FileInputStream fis = openFileInput(fileName);
            BufferedReader in = new BufferedReader(new InputStreamReader(fis));
            line = in.readLine();
            while (line != null) {
                str.add(line);
                line = in.readLine();
            }
            in.close();
            fis.close();
        }
        catch (Throwable t) {
        }
        return str;
    }

    public void writeFile(String fileName, List<String> str, String newString){
        int index;

        //Removes the oldest ip to make space for the new one
        while (str.size() >= 5) {
            str.remove(0);
        }
        str.add(newString);

        try {
            FileOutputStream fos = openFileOutput(fileName, Context.MODE_PRIVATE);

            for(index = 0; index < str.size(); index ++) {
                fos.write(str.get(index).getBytes());
                fos.write(System.getProperty("line.separator").getBytes());
            }
            fos.close();
        }
        catch (Throwable t) {

        }
    }
}
