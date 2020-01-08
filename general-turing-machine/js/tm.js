"use strict";
/*
Created by:emon100
通用图灵机
 */
/*
我现在想做的内容：
    图灵机储存规则
    运行的一个循环，读头获得纸带内容，给到图灵机，图灵机将操作给到读头，读头操作纸带。
 */

/*
测试把所有的1换成0，把所有的0换成1，遇到B之后回去把左边所有符号换成w
 */

let app = {
    tape : ['B','B'],//model
    head : {
        index : 1,
        currentState : null
    },
    ruleTable :{//规则表
        states : new Map(),
        initial: null,
        final: null
    },

    moves : [],
    preset1(){
        this.tape=['B','0','0','1','1','0','0','1','1','0','B'];
        this.ruleTable.initial = "q0";
        this.ruleTable.final = "q2";
        this.head.index=1;
        this.head.currentState=this.ruleTable.initial;


        var a = new Map();
        this.ruleTable.states.set("q0",a);
        this.ruleTable.states.get('q0').set('0',{nextState:"q0",write:"1",move:1});
        this.ruleTable.states.get("q0").set('1',{nextState:"q0",write:"0",move:1});
        this.ruleTable.states.get("q0").set('B',{nextState:"q1",write:"B",move:-1});
        var b = new Map([['0',{nextState:"q1",write:"w",move:-1}]]);
        this.ruleTable.states.set("q1",b);
        this.ruleTable.states.get("q1").set('1',{nextState:"q1",write:"w",move:-1});
        this.ruleTable.states.get("q1").set('B',{nextState:"q2",write:"B",move:1});
    },
    preset2(){
        this.ruleTable.initial = "q0";
        this.ruleTable.final = "q2";
        this.head.index=1;
        this.head.currentState=this.ruleTable.initial;


        var a = new Map();
        this.ruleTable.states.set("q0",a);
        this.ruleTable.states.get('q0').set('0',{nextState:"q0",write:"1",move:1});
        this.ruleTable.states.get("q0").set('1',{nextState:"q0",write:"0",move:1});
        this.ruleTable.states.get("q0").set('B',{nextState:"q1",write:"B",move:-1});
        var b = new Map([['0',{nextState:"q1",write:"w",move:-1}]]);
        this.ruleTable.states.set("q1",b);
        this.ruleTable.states.get("q1").set('1',{nextState:"q1",write:"w",move:-1});
        this.ruleTable.states.get("q1").set('B',{nextState:"q2",write:"B",move:1});
    },
    start() {
        this.moves=[];
        this.preset1();

        this.updateView();
    },
    updateStatesView: function () {
        $("#states-list").empty();
        this.ruleTable.states.forEach((v,k) => {
            var f = function (value, key,map){
                let ol = document.createElement("ol");
                ol.innerText = "δ("+k+" , "+key+") => δ("+value.nextState+" , "+value.write+" , "+value.move+")";
                $("#states-list").append(ol);
            };
            v.forEach(f);
        });
    },

    updateView(){
        this.updateStatesView();
        this.displayTape();
        this.displayHead();
    },
    displayHead(){//viewer刷新读写头
        $(".row2").empty();
        this.tape.forEach(function() {
            let col2 = document.createElement("th");
            col2.innerText;
            $('.row2').append(col2);
        });
        $(".row2").children()[this.head.index].innerText='▲';//设置箭头
    },
    displayTape() {//viewer刷新纸带
        var tape=this.tape;
        $(".row1").empty();
        tape.forEach(function(char){
            let col1=document.createElement("th");
            col1.innerText=char;
            $('.row1').append(col1);
        });
    },
    putInputOnTape(){//controller
        this.clearTape();
        this.tape.pop();
        this.tape = this.tape.concat($('#add').val().split(''));
        this.tape.push('B');
        this.updateView();
    },
    clearTape(){
        this.tape = ['B','B'];
    },
    addTransitionFunction(){
        alert("Haven't done yet")
    },
    parse(){
        var initial = $("#start").val();
        var final = $("#end").val;
        var currentstate = $(".current-state").val();
        var tapeSymbol = $(".tape-symbol").val();
        var currentstate = $(".current-state").val();
        var tapeSymbol = $(".tape-symbol").val();
       console.log(initial);
        console.log(final);
        console.log(currentstate);
        console.log(tapeSymbol);

    },
    oneStepBackward() {//回退一步
        if(this.moves.length!=0){
            var M =this.moves.pop();
            this.head.index=M.index;
            this.head.currentState=M.state;
            this.tape[this.head.index]=M.symbol;
            this.updateView();
        }else{
            alert("can't go backward anymore");
        }
    },
    oneStepForward(){//进行一步
        if(this.head.currentState==this.ruleTable.final){//终止
            $('#modal-container-1').modal('show');
            return;
        }
        if(this.head.index<0){//读头左端越界
            $('#modal-container-3').modal('show');
            return;
        }
        var currentSymbol = this.tape[this.head.index];
        this.head.currentState = this.head.currentState == null?this.ruleTable.initial:this.head.currentState;
        var currentState = this.head.currentState;
        var rule = this.ruleTable.states.get(currentState);

        if(rule == undefined) {//无此状态
            $('#modal-container-2').modal('show');
            return;
        }else {//此状态无读到此字符的操作
            rule = rule.get(currentSymbol);
            if(rule == undefined){
                $('#modal-container-2').modal('show');
                return;
            }
            var M={state:this.head.currentState,symbol:this.tape[this.head.index],index:this.head.index};//当前状态备份
            this.moves.push(M);//当前状态入栈
            this.head.currentState = rule.nextState;//设置状态
            this.tape[this.head.index]=rule.write;//写入纸带
            this.head.index += rule.move;//移动读头
            if(this.head.index==this.tape.length){this.tape.push('B');}//向右延伸
            if(this.head.index<0){
                $('#modal-container-3').modal('show');
                return;
            }
        }
        this.updateView();
    }
};

$(function () {//保证DOM加载再开始程序
    $('#submit').click(function () {
        app.start();
        app.putInputOnTape();
    });
    $('#onstepback').click(function () {
        app.oneStepBackward();
    });
    $('#onstepforward').click(function () {
        app.oneStepForward();
    });
    $('#addTransitionFunction').click(function () {
        app.parse();
    })
    app.start();
});


