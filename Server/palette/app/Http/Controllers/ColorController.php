<?php

namespace App\Http\Controllers;

use Illuminate\Http\Request;
use App\Models\Color;
use App\Models\User;

class ColorController extends Controller
{
    /**
     * DBの色情報を全部取得
     *
     * @return \Illuminate\Http\Response
     */
    public function index()
    {
        $colors = Color::get(); //SELECT * FROM color
        return response($colors, 200);
    }

    /**
     * Show the form for creating a new resource.
     *
     * @return \Illuminate\Http\Response
     */
    public function create()
    {
        //
    }

    /**
     * 色情報を登録する
     *
     * @param  \Illuminate\Http\Request  $request
     * @return \Illuminate\Http\Response
     */
    public function store(Request $request)
    {
        // return response($request, 201);
        //ユーザ複数に対応してない
        //SELECT * FROM users WHERE leafony_name = $request['leafony_name']
        $user = User::where('leafony_name', $request['leafony_name'])->first();
        //INSERT INTO colors(...) VALUES
        $color =  Color::create([
            'user_id' => $user->id,
            'x' => $request['x'],
            'y' => $request['y'],
            'red' => $request['red'],
            'green' => $request['green'],
            'blue' =>  $request['blue'],
            // 'clear' => $request['clear'],
            'radius' => $request['radius']
        ]);
        return response(['id' => $color->id], 201);
    }

    /**
     * Display the specified resource.
     *
     * @param  int  $id
     * @return \Illuminate\Http\Response
     */
    public function show($id)
    {
        //
    }

    /**
     * Show the form for editing the specified resource.
     *
     * @param  int  $id
     * @return \Illuminate\Http\Response
     */
    public function edit($id)
    {
        //
    }

    /**
     * Update the specified resource in storage.
     *
     * @param  \Illuminate\Http\Request  $request
     * @param  int  $id
     * @return \Illuminate\Http\Response
     */
    public function update(Request $request, $id)
    {
        //
    }

    /**
     * Remove the specified resource from storage.
     *
     * @param  int  $id
     * @return \Illuminate\Http\Response
     */
    public function destroy($id)
    {
        //
    }
}
