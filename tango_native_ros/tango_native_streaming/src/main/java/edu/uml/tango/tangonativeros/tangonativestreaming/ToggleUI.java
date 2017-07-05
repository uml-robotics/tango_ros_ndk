package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.content.Context;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;

import java.util.ArrayList;
import java.util.List;
//extends SettingsActivity
class ToggleUI {
    boolean isNew = false;
    Spinner spinner;
    EditText editTxt;
    Button toggleBtn;
    List<String> dataStr = new ArrayList<String>();
    String fileName;
    String newDataButtonString;

    void initSpinner(Context context){
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(context, android.R.layout.simple_spinner_item, reverseList(dataStr));
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinner.setAdapter(adapter);
    }

    String dataFromUser(){
        String newData;
        if(!isNew) {
            newData = spinner.getSelectedItem().toString();
        }
        else {
            newData = editTxt.getText().toString();
        }
        return newData;
    }

    void toggleBtns(){
        if(!isNew) {
            isNew = true;
            spinner.setVisibility(View.GONE);
            toggleBtn.setText(R.string.backToList);
            editTxt.setVisibility(View.VISIBLE);
        }
        else{
            isNew = false;
            spinner.setVisibility(View.VISIBLE);
            toggleBtn.setText(newDataButtonString);
            editTxt.setVisibility(View.GONE);
        }

    }

    private List<String> reverseList(List<String> str){
        List<String> revList = new ArrayList<String>();
        int index;
        //Reverse list so that the most recent data from editTxt is on top of spinner
        for(index = str.size() - 1; index >= 0; index--){
            revList.add(str.get(index));
        }
        return revList;
    }
}
