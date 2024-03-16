`include "uvm_macros.svh"

package my_pkg;
import uvm_pkg::*; 
///////////////////////////////////////////////////////////////////////////
class uart_tx extends uvm_sequence_item;
 
    
  bit    i_TX_DV;     
  rand logic [7:0] i_TX_Byte;
  //reg         r_TX_Serial;
  //reg         r_TX_Active;
   
  reg [7:0]   r_RX_Byte;
  reg         r_RX_DV;
  //reg         o_RX_Byte;
  
       //Register with factory for dynamic creation
  `uvm_object_utils(uart_tx)
  
   
  function new (string name = "uart_tx");
      super.new(name);
   endfunction

   function string convert2string();
     return $psprintf("i_TX_DV=%0h  i_TX_Byte=%0h  " ,i_TX_DV,i_TX_Byte);
   endfunction
  
   function string convert3string();
     return $psprintf("i_TX_Byte=%0h  ",i_TX_Byte);
   endfunction
  
   function string convert33string();
     return $psprintf("  r_RX_Byte=%0h ",r_RX_Byte);
   endfunction
  
   function string convert4string();
     return $psprintf("r_RX_Byte=%0h ",r_RX_Byte);
   endfunction
  
  function string convert5string();
    return $psprintf("i_TX_Byte=%0h ",i_TX_Byte);
   endfunction

endclass: uart_tx

//////////////////////////////////////////////////////////////////////

class uart_tx_seq extends uvm_sequence#(uart_tx);

  `uvm_object_utils(uart_tx_seq)
       uart_tx uart_trans;
  
  function new(string name ="");
    super.new(name);
  endfunction


  //Main Body method that gets executed once sequence is started
  task body();
     
     //Create 10 random uart read/write transaction and send to driver
    uart_trans = uart_tx::type_id::create("uart_trans");
      //apb_rw::type_id::create(.name("rw_trans"),.contxt(get_full_name()));
    for(int i=0;i<2;i++) begin
      start_item(uart_trans);
   // assert (uart_trans.randomize());
     assert(uart_trans.randomize() with { i_TX_Byte == 8'h5a;});
      
      finish_item(uart_trans);
      
        
     end
  endtask
  
endclass

/////////////////////////////////////////////////////////////////////////

class uart_tx_sequencer extends uvm_sequencer #(uart_tx);

   `uvm_component_utils(uart_tx_sequencer)
 
   function new(input string name, uvm_component parent=null);
      super.new(name, parent);
   endfunction : new

endclass : uart_tx_sequencer

/////////////////////////////////////////////////////////////////////////

class uart_tx_config extends uvm_object;

   `uvm_object_utils(uart_tx_config)
   virtual UART_IF vif;

  function new(string name="uart_tx_config");
     super.new(name);
  endfunction

endclass


typedef uart_tx_config;
typedef uart_tx_agent;
typedef uart_rx_agent;
///////////////////////////////////////////////////////////////////////

class uart_tx_driver extends uvm_driver #(uart_tx);
  
  `uvm_component_utils(uart_tx_driver)
  
  virtual UART_IF vif ;
          //uart_tx_config cfg_tx;
  function new(input string name, uvm_component parent=null);
      super.new(name, parent);
   endfunction : new
  
   function void build_phase(uvm_phase phase);
    /* uart_tx_agent agent;
     //super.build_phase(phase);
     if ($cast(agent, get_parent()) && agent != null) begin
         vif = agent.vif;
     end
     else begin*/
       if (!uvm_config_db#(virtual UART_IF)::get(this, "", "vif", vif)) begin
         `uvm_fatal("UART/DRV/NOVIF", "No virtual interface specified for this driver instance")
         end
    // end
   endfunction
  
  virtual task run_phase(uvm_phase phase);
    super.run_phase(phase);
    
     vif.i_TX_Byte=0;
       vif.i_TX_DV=0;
       vif.o_TX_Serial=0;
     forever begin
       uart_tx tr;
        //@ (posedge vif.i_Clock);
      
       
    @ (posedge vif.i_Clock);
       //First get an item from sequencer
       seq_item_port.get_next_item(tr);
      // @ (this.vif.i_Clock);
       uvm_report_info("UART_DRIVER ", $psprintf("Got Transaction %s",tr.convert2string()));
    
    @ (this.vif.i_Clock);
       
           vif.i_TX_Byte=tr.i_TX_Byte;
           vif.i_TX_DV=1;
       
    @ (this.vif.i_Clock);
          vif.i_TX_DV=0;
       
       
       
       seq_item_port.item_done();
       
     end
  endtask
endclass:uart_tx_driver
/////////////////////////////////////////////////
  
  class uart_tx_monitor extends uvm_monitor;
     virtual UART_IF vif;

  //Analysis port -parameterized to apb_rw transaction
  ///Monitor writes transaction objects to this port once detected on interface
    uvm_analysis_port#(uart_tx) item_collected_port_tx;

  //config class handle
  uart_tx_config cfg;

  `uvm_component_utils(uart_tx_monitor)

   function new(string name, uvm_component parent = null);
     super.new(name, parent);
     item_collected_port_tx = new("item_collected_port_tx", this);
   endfunction: new

   //Build Phase - Get handle to virtual if from agent/config_db
   virtual function void build_phase(uvm_phase phase);
     uart_tx_agent agent;
     if ($cast(agent, get_parent()) && agent != null) begin
       vif = agent.vif;
     end
     else begin
       virtual UART_IF tmp;
       if (!uvm_config_db#(virtual UART_IF)::get(this, "", "UART_IF", tmp)) begin
         `uvm_fatal("APB/MON/NOVIF", "No virtual interface specified for this monitor instance")
       end
       vif = tmp;
     end
   endfunction
    
 virtual task run_phase(uvm_phase phase);
    // super.run_phase(phase);
     //  super.run_phase(phase);
     uart_tx tr;
     tr =uart_tx::type_id::create("tr");
   //tr.o_TX_Serial=0;
    forever begin
    //  uart_tx tr;
      @(this.vif.i_Clock);
      if(vif.i_TX_DV==1)begin  
        $display("i_TX_DV is high");
        // tr.r_TX_Serial = vif.o_TX_Serial;
        // tr.r_TX_Active = vif.o_TX_Active;
           tr.i_TX_Byte = vif.i_TX_Byte;
          item_collected_port_tx.write(tr);
        uvm_report_info("UART_TX_MONITER ", $psprintf("Got Transaction %s",tr.convert5string()));
      
         
     // @ (this.vif.i_Clock);
      // item_collected_port_tx.write(tr);
      end
      end
   
 endtask
      
      endclass: uart_tx_monitor
///////////////////////////////////////////////////////      
      
      class uart_tx_agent extends uvm_agent;

   //Agent will have the sequencer, driver and monitor components for the APB interface
   uart_tx_sequencer sqr;
   uart_tx_driver drv;
   uart_tx_monitor mon;

   virtual UART_IF  vif;

   `uvm_component_utils_begin(uart_tx_agent)
      `uvm_field_object(sqr, UVM_ALL_ON)
      `uvm_field_object(drv, UVM_ALL_ON)
      `uvm_field_object(mon, UVM_ALL_ON)
   `uvm_component_utils_end
   
   function new(string name, uvm_component parent = null);
      super.new(name, parent);
   endfunction

   //Build phase of agent - construct sequencer, driver and monitor
   //get handle to virtual interface from env (parent) config_db
   //and pass handle down to srq/driver/monitor
   virtual function void build_phase(uvm_phase phase);
      sqr = uart_tx_sequencer::type_id::create("sqr", this);
      drv = uart_tx_driver::type_id::create("drv", this);
      mon = uart_tx_monitor::type_id::create("mon", this);
      
     if (!uvm_config_db#(virtual UART_IF)::get(this, "", "vif", vif)) begin
         `uvm_fatal("APB/AGT/NOVIF", "No virtual interface specified for this agent instance")
      end
     uvm_config_db#(virtual UART_IF)::set( this, "sqr", "vif", vif);
     uvm_config_db#(virtual UART_IF)::set( this, "drv", "vif", vif);
     uvm_config_db#(virtual UART_IF)::set( this, "mon", "vif", vif);
   endfunction: build_phase

   //Connect - driver and sequencer port to export
   virtual function void connect_phase(uvm_phase phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
     uvm_report_info("uart_tx_agent::", "connect_phase, Connected driver to sequencer");
     endfunction
        
      endclass
/////////////////////////////////////////////////////////
      
      class uart_rx_monitor extends uvm_monitor;
     virtual UART_IF vif;
         uart_tx tr;

  //Analysis port -parameterized to apb_rw transaction
  ///Monitor writes transaction objects to this port once detected on interface
        uvm_analysis_port#(uart_tx) item_collected_port_rx;

  //config class handle
 // uart_tx_config cfg;

        `uvm_component_utils(uart_rx_monitor)

   function new(string name, uvm_component parent = null);
     super.new(name, parent);
     item_collected_port_rx = new("item_collected_port_rx", this);
   endfunction: new

   //Build Phase - Get handle to virtual if from agent/config_db
   virtual function void build_phase(uvm_phase phase);
     uart_rx_agent agent;
     if ($cast(agent, get_parent()) && agent != null) begin
       vif = agent.vif;
     end
     else begin
       virtual UART_IF tmp;
     if (!uvm_config_db#(virtual UART_IF)::get(this, "", "UART_IF", vif)) begin
       `uvm_fatal("UART/MON/NOVIF", "No virtual interface specified for this monitor instance")
       end
       vif = tmp;
     end
   endfunction
        
        
        
    
 virtual task run_phase(uvm_phase phase);
  //   super.run_phase(phase);
     uart_tx tr;
     tr =uart_tx::type_id::create("tr");
    forever begin

      @(posedge vif.i_Clock)
      if(vif.o_RX_DV==1) begin
         tr.r_RX_Byte = vif.o_RX_Byte;
         //vif.o_RX_DV = 1'b1;
         //tr.r_RX_DV = vif.o_RX_DV; 
      
      uvm_report_info("UART_RX_MONITER ", $psprintf("Got Transaction %s",tr.convert4string()));
      
        @(posedge vif.i_Clock)
      item_collected_port_rx.write(tr);
         //vif.o_RX_DV = 1'b0;
      end 
    end
      
      endtask
      
      endclass: uart_rx_monitor
////////////////////////////////////////////////////////
      
        class uart_rx_agent extends uvm_agent;

   //Agent will have the sequencer, driver and monitor components for the APB interface
   
   uart_rx_monitor mon;
   uart_tx_sequencer  sqr;
   uart_tx_driver drv;
   virtual UART_IF  vif;
          
    uvm_active_passive_enum is_active = UVM_PASSIVE;

    `uvm_component_utils_begin (uart_rx_agent)   
          `uvm_field_object(mon, UVM_ALL_ON)
          `uvm_field_object(sqr, UVM_ALL_ON)
          `uvm_field_object(drv, UVM_ALL_ON)
   `uvm_component_utils_end
   
   function new(string name, uvm_component parent = null);
      super.new(name, parent);
   endfunction

   //Build phase of agent - construct sequencer, driver and monitor
   //get handle to virtual interface from env (parent) config_db
   //and pass handle down to srq/driver/monitor
   virtual function void build_phase(uvm_phase phase);
     
     if(is_active==UVM_ACTIVE) begin
      sqr = uart_tx_sequencer::type_id::create("sqr", this);
      drv = uart_tx_driver::type_id::create("drv", this);
     end
      mon = uart_rx_monitor::type_id::create("mon", this);
              
     if (!uvm_config_db#(virtual UART_IF)::get(this, "", "vif", vif)) begin
         `uvm_fatal("APB/AGT/NOVIF", "No virtual interface specified for this agent instance")
      end
     if (is_active==UVM_ACTIVE) begin
     uvm_config_db#(virtual UART_IF)::set( this, "sqr", "vif", vif);
     uvm_config_db#(virtual UART_IF)::set( this, "drv", "vif", vif);
       
     end
         
     uvm_config_db#(virtual UART_IF)::set( this, "mon", "vif", vif);
    
     
   endfunction: build_phase
          
   //Connect - driver and sequencer port to export
   virtual function void connect_phase(uvm_phase phase);
     if (is_active==UVM_ACTIVE) begin
      drv.seq_item_port.connect(sqr.seq_item_export);
      end
    
     endfunction

      
      endclass : uart_rx_agent
///////////////////////////////////////////////////////////////
      
   class uart_scoreboard extends uvm_scoreboard;

  `uvm_component_utils(uart_scoreboard);
                        
     uvm_tlm_analysis_fifo #(uart_tx) tx_fifo;
     uvm_tlm_analysis_fifo #(uart_tx) rx_fifo;
  
  uart_tx  input_sequence_item;
  uart_tx output_sequence_item;
                 
  function new(string name,uvm_component parent);
    super.new(name,parent);
  endfunction  
               
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    tx_fifo = new("tx_fifo",this);
    rx_fifo = new("rx_fifo",this);
  endfunction
  
   task run_phase(uvm_phase phase);
   uart_tx input_sequence_item, output_sequence_item;
  forever begin
    `uvm_info("scoreboard run task", "Waiting for EXPECTED values",UVM_MEDIUM)
    tx_fifo.get(input_sequence_item);
    `uvm_info("scoreboard run task", "Waiting for ACTUAL values",UVM_MEDIUM)
    rx_fifo.get(output_sequence_item);
    
    //`uvm_info("scoreboard ",$sformatf( "input_sequence_item=%h output_sequence_item=%h",input_sequence_item.convert3string(), output_sequence_item.convert33string()),UVM_MEDIUM)
    
    `uvm_info("scoreboard ",$sformatf( "input_sequence_item=%h output_sequence_item=%h",input_sequence_item.i_TX_Byte, output_sequence_item.r_RX_Byte),UVM_MEDIUM)

    ///////////////////////////////////////////////////////////////////
    
    if (output_sequence_item.r_RX_Byte == input_sequence_item.i_TX_Byte) begin
    
      //////////////////////////////////////////////////////////////////  
      `uvm_info ("ACTUAL = EXPECTED ", $sformatf("ACTUAL=%s EXPECTED=%s \n",output_sequence_item.convert33string(),input_sequence_item.convert3string()), UVM_NONE);
      $display("PPPPPPP    AA     SSSSSSS   SSSSSSS\nPP   PP  AA  AA   SS        SS\nPPPPPP  AAAAAAAA  SSSSSSS   SSSSSSS\nPP      AA    AA       SS        SS\nPP      AA    AA  SSSSSSS   SSSSSSS");
    end  
else begin
  `uvm_error("ACTUAL != EXPECTED", $sformatf("ACTUAL=%s EXPECTED=%s \n",output_sequence_item.convert33string(), input_sequence_item.convert3string()));
  
$display("FFFFFFF    AA     IIIIIIII   LL\nFF       AA  AA      II      LL\nFFFFF   AAAAAAAA     II      LL\nFF      AA    AA     II      LL\nFF      AA    AA  IIIIIIII   LLLLLLL");
end
end
endtask
 endclass:uart_scoreboard

  
//endclass:fifo_scoreboard

/////////////////////////////////////////////////////////////////////////////
      
/////////////////////////////////////////////////////////////////////////////
class uart_env  extends uvm_env;
  virtual UART_IF  vif;
   `uvm_component_utils(uart_env);
     uart_tx_agent agent_tx;
     uart_rx_agent agent_rx;
  uart_scoreboard scoreboard;
   //ENV class will have agent as its sub component
  // apb_agent  agt;
   //virtual interface for APB interface
  

   function new(string name, uvm_component parent = null);
      super.new(name, parent);
   endfunction

   //Build phase - Construct agent and get virtual interface handle from test  and pass it down to agent
   function void build_phase(uvm_phase phase);
     agent_tx = uart_tx_agent::type_id::create("agent_tx", this);
     agent_rx = uart_rx_agent::type_id::create("agent_rx", this);
    scoreboard =uart_scoreboard::type_id::create("scoreboard",this);    
     if (!uvm_config_db#(virtual UART_IF)::get(this, "", "vif", vif)) begin
         `uvm_fatal("APB/AGT/NOVIF", "No virtual interface specified for this env instance")
     end
     uvm_config_db#(virtual UART_IF)::set( this, "agent_tx", "vif", vif);
     uvm_config_db#(virtual UART_IF)::set( this, "agent_rx", "vif", vif);
   endfunction: build_phase
   
  function void connect_phase(uvm_phase phase);
    agent_tx.mon.item_collected_port_tx.connect(scoreboard.tx_fifo.analysis_export);
    `uvm_info("environment","TX monitor  connected scoreboard",UVM_LOW);    
    agent_rx.mon.item_collected_port_rx.connect(scoreboard.rx_fifo.analysis_export);
    `uvm_info("environment","RX monitor connected to scoreboard",UVM_LOW);
  endfunction  
              
              endclass:uart_env
        
 //////////////////////////////////////////////////////////////////////////
        class uart_base_test extends uvm_test;

  //Register with factory
          `uvm_component_utils(uart_base_test);
 uart_env  env;
  uart_tx_config cfg;
           uart_tx_seq uart_seq;
  virtual UART_IF vif;
  
          function new(string name = "uart_base_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  //Build phase - Construct the cfg and env class using factory
  //Get the virtual interface handle from Test and then set it config db for the env component
  function void build_phase(uvm_phase phase);
    cfg = uart_tx_config::type_id::create("cfg", this);
    env = uart_env::type_id::create("env", this);
    //
    if (!uvm_config_db#(virtual UART_IF)::get(this, "", "vif", vif)) begin
      `uvm_fatal("UART/DRV/NOVIF", "No virtual interface specified for this test instance")
    end 
    uvm_config_db#(virtual UART_IF)::set( this, "env", "vif", vif);
  endfunction

  //Run phase - Create an abp_sequence and start it on the apb_sequencer
  task run_phase( uvm_phase phase );
    uart_tx_seq uart_seq;
    uart_seq = uart_tx_seq::type_id::create("uart_seq");
    phase.raise_objection( this, "Starting uart_tx_seqin main phase" );
    `uvm_info ("UART/TEST","Phase raise Objection",UVM_LOW)
    $display("%t Starting sequence uart_seq run_phase",$time);
    uart_seq.start(env.agent_tx.sqr);
    #100000ns;
    phase.drop_objection( this , "Finished uart_seq in main phase" );
    `uvm_info ("UART/TEST","Phase drop Objection",UVM_LOW)
    $display("%t ending sequence uart_seq run_phase",$time);
  endtask: run_phase
  
  
endclass  
              endpackage
 ////////////////////////////////////////////////////////
              
 `timescale 1ns/10ps

//`include "UART_TX.v"

module UART_TB ();
  
   import uvm_pkg::*;
   import my_pkg::*;
 
  UART_IF vif();

  // Testbench uses a 25 MHz clock
  // Want to interface to 115200 baud UART
  // 25000000 / 115200 = 217 Clocks Per Bit.
  parameter c_CLOCK_PERIOD_NS = 40;
  parameter c_CLKS_PER_BIT    = 217;
  parameter c_BIT_PERIOD      = 8600;
  
  bit i_Clock;
  wire y;


  UART_RX #(c_CLKS_PER_BIT) UART_RX_Inst
  (.i_Clock(vif.i_Clock),
   .i_RX_Serial(y),
     .o_RX_DV(vif.o_RX_DV),
     .o_RX_Byte(vif.o_RX_Byte)
     );
  
  UART_TX #(c_CLKS_PER_BIT) UART_TX_Inst
     (.i_Clock(vif.i_Clock),
     .i_TX_DV(vif.i_TX_DV),
    .i_TX_Byte(vif.i_TX_Byte),
     .o_TX_Active(vif.o_TX_Active),
      .o_TX_Serial(y),
     .o_TX_Done()
     );
  
  
   always begin
     #(c_CLOCK_PERIOD_NS/2) i_Clock = !i_Clock;
    
  
//     $display("top_i_Clock=%d ",i_Clock);
     
   end
  initial begin 
      
    //Pass this physical interface to test top (which will further pass it down to env->agent->drv/sqr/mon
    uvm_config_db#(virtual UART_IF)::set(null,"*", "vif", vif);
    
    `uvm_info("top","uvm_config_db set for uvm_test_top",UVM_LOW);
    //Call the test - but passing run_test argument as test class name
    //Another option is to not pass any test argument and use +UVM_TEST on command line to sepecify which test to run
    run_test("uart_base_test");
  end
  
    
 // initial $dumpvars(0, UART_TB);


   assign vif.i_Clock=i_Clock;
  
  
  initial  begin 
    $dumpfile("dump.vcd");
    $dumpvars(0, UART_TB);
  #2000ns;
  end
  
endmodule
