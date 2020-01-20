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
        currentState : null,
        moves : [],
        init(){
            this.index=1;
            this.currentState=null;
            this.moves=[];
        }
    },
    ruleTable :{//规则表
        description: null,
        states : new Map(),
        initial: null,
        final: null,
        init(){
            this.description=null;
            this.states =new Map();
            this.initial=null;
            this.final=null;
        },
        putState(current,symbol,next,write,move){//放入新的状态转移函数
            if(this.states.get(current)===undefined){
                let a = new Map([[symbol,{nextState:next,write:write,move:move}]]);
                this.states.set(current,a);
            }else{
                this.states.get(current).set(symbol,{nextState:next,write:write,move:move});
            }
        }
    },

    setPreset1(){//预设1：计算按位取反
        this.ruleTable.initial = "q0";
        this.ruleTable.final = "q1";
        this.tape=['B','0','0','1','1','0','0','1','1','0','B'];

        this.ruleTable.putState('q0','0','q0',"1",1);
        this.ruleTable.putState("q0",'1',"q0","0",1);
        this.ruleTable.putState("q0",'B',"q1","B",0);
        },
    setPreset2(){//预设2：计算一进制m-n
        this.ruleTable.initial = "q0";
        this.ruleTable.final = "q6";
        this.tape=['B','0','0','0','0','0','1','0','0','B'];

        this.ruleTable.putState('q0','0','q1','B',1);
        this.ruleTable.putState('q0','1','q5','B',1);
        this.ruleTable.putState('q1','0','q1','0',1);
        this.ruleTable.putState('q1','1','q2','1',1);
        this.ruleTable.putState('q2','1','q2','1',1);
        this.ruleTable.putState('q2','0','q4','1',-1);
        this.ruleTable.putState('q2','B','q3','B',-1);
        this.ruleTable.putState('q3','0','q3','0',-1);
        this.ruleTable.putState('q3','1','q3','B',-1);
        this.ruleTable.putState('q3','B','q6','0',0);
        this.ruleTable.putState('q4','0','q4','0',-1);
        this.ruleTable.putState('q4','1','q4','1',-1);
        this.ruleTable.putState('q4','B','q0','B',1);
        this.ruleTable.putState('q5','0','q5','B',1);
        this.ruleTable.putState('q5','1','q5','B',1);
        this.ruleTable.putState('q5','B','q6','B',0);
    },

    /*开始的函数*/
    start() {
        this.initAll();
        this.setPreset1();
        this.updateView();
    },
    initAll(){
        this.head.init();
        this.clearTape();
        this.ruleTable.init();
    },
    /*刷新所有状态*/
    updateView(){
        this.updateStatesView();
        this.updateTapeView();
        this.updateHeadView();
    },
    /*刷新状态页面状态*/
    updateStatesView() {
        $("#start").val(this.ruleTable.initial);
        $("#end").val(this.ruleTable.final);

        let ol3 = document.createElement("ol");
        ol3.innerText = "当前状态:" + (this.head.currentState == null ? "未启动" : this.head.currentState);
        $('#states-indicator').empty().append(ol3);

        let statesList = $("#states-list");
        statesList.empty();
        let ol2 = document.createElement("ol");
        ol2.innerText="初始状态:"+(this.ruleTable.initial==null?"为空":this.ruleTable.initial)+"\n结束状态:"+(this.ruleTable.final==null?"为空":this.ruleTable.final);
        statesList.append(ol2);
        this.ruleTable.states.forEach((v,k) => {
            let f = function (value, key){
                let ol = document.createElement("ol");
                ol.innerText = "δ("+k+" , "+key+") = δ("+value.nextState+" , "+value.write+" , "+(value.move === 1?'R':value.move === -1?'L':'S')+")";
                statesList.append(ol);
            };
            v.forEach(f);
        });
    },
    /*刷新读头状态*/
    updateHeadView(){//viewer刷新读写头
        let row2 =$(".row2");
            row2.empty();
        this.tape.forEach(function() {
            let col2 = document.createElement("th");
            col2.innerText;
            row2.append(col2);
        });
        row2.find('th').eq(this.head.index).text('▲');
    },
    /*刷新纸带状态*/
    updateTapeView() {//viewer刷新纸带
        let tape=this.tape;
        let row1 =$(".row1");
        row1.empty();
        tape.forEach(function(symbol){
            let col1=document.createElement("th");
            col1.innerText=symbol;
            row1.append(col1);
        });
    },
    /*将设置的内容放入纸带*/
    putInputOnTape(){//controller
        this.clearTape();
        this.tape.pop();
        this.tape = this.tape.concat($('#add').val().split(''));
        this.tape.push('B');
    },
    /*清空纸带*/
    clearTape(){
        this.tape = ['B','B'];
    },
    /*修改起始结束状态*/
    changeInitFinal(){
        let initial = $("#start").val();
        let final = $("#end").val();
        this.ruleTable.initial=initial;
        this.ruleTable.final=final;
    },
    /*修改移动函数*/
    changeTransitionFunctions(){

        let currentState = $("#current-state").val();
        let tapeSymbol = $("#tape-symbol").val();
        let nextState = $("#next-state").val();
        let writeSymbol = $("#write-symbol").val();
        let mo=$("#move");
        let move = mo.val()==='L'?-1:mo.val()==='R'?1:0;

        this.ruleTable.putState(currentState,tapeSymbol,nextState,writeSymbol,move);

    },
    /*重新开始*/
    restartMachine(){
        while(this.head.moves.length!==0) {
            let M = this.head.moves.pop();
            this.head.index=M.index;
            this.tape[this.head.index] = M.symbol;
        }
        this.head.init();
    },
    /*回退一步*/
    oneStepBackward() {//回退一步
        if(this.head.moves.length!==0){
            let M =this.head.moves.pop();
            this.head.index=M.index;
            this.head.currentState=M.state;
            this.tape[this.head.index]=M.symbol;
        }else{
            $('#modal-container-4').modal('show');
        }
    },
    /*前进一步*/
    oneStepForward(){//进行一步
        if(this.head.currentState===this.ruleTable.final){//终止
            $('#modal-container-1').modal('show');
            return;
        }
        if(this.head.index<0){//读头左端越界
            $('#modal-container-3').modal('show');
            return;
        }
        const currentSymbol = this.tape[this.head.index];
        this.head.currentState = this.head.currentState == null?this.ruleTable.initial:this.head.currentState;
        const currentState = this.head.currentState;
        let rule = this.ruleTable.states.get(currentState);

        if(rule === undefined) {//无此状态
            $('#modal-container-2').modal('show');
        }else {//此状态无读到此字符的操作
            rule = rule.get(currentSymbol);
            if(rule === undefined){
                $('#modal-container-2').modal('show');
                return;
            }
            const M={state:this.head.currentState,symbol:this.tape[this.head.index],index:this.head.index};//当前状态备份
            this.head.moves.push(M);//当前状态入栈
            this.head.currentState = rule.nextState;//设置状态
            this.tape[this.head.index]=rule.write;//写入纸带
            this.head.index += rule.move;//移动读头
            if(this.head.index===this.tape.length){this.tape.push('B');}//向右延伸
            if(this.head.index<0){
                $('#modal-container-3').modal('show');
            }
        }
    },
    /*将目前的纸带和状态函数序列化*/
    serialize(){
        let tapeJson=JSON.stringify(this.tape);
        let ruleJson=JSON.stringify(this.ruleTable.states);
        console.log(tapeJson);
        console.log(ruleJson);
    }
};

//保证DOM加载再开始程序
$(function () {
    $('#submit').on('click',function () {
        app.head.init();
        app.putInputOnTape();
        app.updateHeadView();
        app.updateTapeView();
    });
    $('#restartMachine').on('click',function () {
        app.restartMachine();
        app.updateView();
    });
    $('#stepBackward').on('click',function () {
        app.oneStepBackward();
        app.updateView();
    });
    $('#stepForward').on('click',function () {
        app.oneStepForward();
        app.updateView();
    });
    $('#change-initial-final').on('click',function () {
        app.changeInitFinal();
        app.updateStatesView();
    });
    $('#updateTransitionFunction').on('click',function () {
        app.changeTransitionFunctions();
        app.updateStatesView();
    });
    $('#preset1').on('click',function () {
        app.initAll();
        app.setPreset1();
        app.updateView();
    });
    $('#preset2').on('click',function () {
        app.initAll();
        app.setPreset2();
        app.updateView();
    });
    $('#customTuringMachine').on('click',function () {
        app.initAll();
        app.updateView();
    });
    app.start();
});


