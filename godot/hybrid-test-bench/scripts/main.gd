extends Node3D


@onready var hybrid_test_bench: HybridTestBench = $HybridTestBench
@onready var rabbit_mq: Node = $RabbitMQ
@onready var ui: Ui = $UI


func _ready() -> void:
	rabbit_mq.connect("OnMessage", _on_message)

func _on_message(message):
	var data = JSON.parse_string(message)
	print(data)
	
	if data.measurement != "emulator":
		return
	
	if data.fields.force_on:
		hybrid_test_bench.set_actuator_on()
	else:
		hybrid_test_bench.set_actuator_off()
		
	ui.update_vertical_force(data.fields.vertical_force)
	ui.update_horizontal_force(data.fields.horizontal_force)
	ui.update_vertical_displacement(data.fields.vertical_displacement)
	ui.update_horizontal_displacement(data.fields.horizontal_displacement)

	hybrid_test_bench.move_indicator(data.fields.horizontal_displacement, data.fields.vertical_displacement)
