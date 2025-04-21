extends Node3D

@onready var hybrid_test_bench: HybridTestBench = $HybridTestBench
@onready var rabbit_mq: Node = $RabbitMQ

func _ready() -> void:
	rabbit_mq.connect("OnMessage", _on_message)
	#hybrid_test_bench.move_actuator()

func _on_message(message):
	var data = JSON.parse_string(message)
	print(data)
	
	if data.measurement == "emulator":
		#var room_temp_label = %room_temp as Label3D
		#room_temp_label.text = str(snapped(data.fields.t3, 0.01)) + "°C"
		#
		#var heater_label = %heater_label as Label3D
		#heater_label.text = str(snapped(data.fields.average_temperature, 0.01)) + "°C"

		if data.fields.force_on:
			hybrid_test_bench.change_actuator_colour()
		
		#if data.fields.fan_on:
			#var fan_tween = get_tree().create_tween().set_loops(3)
			#fan_tween.tween_property(%fan, "global_rotation", Vector3(TAU, 0, 0), 1).as_relative()
			#
